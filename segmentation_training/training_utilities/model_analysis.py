#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations
from pathlib import Path
from typing import Optional, Tuple, List, Dict
import time
import numpy as np
import torch
import cv2

from training_utilities.utils import save_histogram, k_closest_idx


class ModelAnalyzer:
    """
    Post-training analysis for segmentation:
    """

    # ------------- public API -------------

    def __init__(
        self,
        model: torch.nn.Module,
        dataset,                      # yields (img: (C,H,W), mask: (1,H,W))
        device: torch.device,
        out_dir: Path,
        threshold: float = 0.7,
        top_k: int = 5,
    ):
        self.model = model
        self.dataset = dataset
        self.device = device
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.threshold = float(threshold)
        self.top_k = int(top_k)

        self._columns: List[Tuple[np.ndarray, ...]] = []
        self._costs: List[int] = []
        self._times_ms: List[float] = []

    def run(self) -> Dict[str, float]:
        """Run full analysis, save grids + histograms, return summary metrics."""
        self.model.eval()
        with torch.no_grad():
            for i in range(len(self.dataset)):
                img, mask = self.dataset[i]           # img: (C,H,W) in [0,1]; mask: (1,H,W) {0,1}
                x = img.unsqueeze(0).to(self.device)  # (1,C,H,W)
                pred_1hw, ms = self._forward(x)
                col, cost = self._make_column(img, mask, pred_1hw)
                self._columns.append(col)
                self._costs.append(cost)
                self._times_ms.append(ms)

        # rankings
        costs = np.asarray(self._costs, float)
        times = np.asarray(self._times_ms, float)
        n = len(costs)
        mean, p10, p90 = float(costs.mean()), float(np.percentile(costs, 10)), float(np.percentile(costs, 90))
        order = np.argsort(costs)
        k = min(self.top_k, n)

        def pick(idx): return [self._columns[i] for i in idx]

        top       = pick(order[:k])
        bottom    = pick(order[-k:])
        near_mean = pick(k_closest_idx(costs, mean, k))
        near_p10  = pick(k_closest_idx(costs, p10,  k))
        near_p90  = pick(k_closest_idx(costs, p90,  k))

        row_labels = ["roi:", "mask:", "difference:", "predicted:", "ellipse fit:"]
        self._render_grid("top_grid.png",       top,       row_labels)
        self._render_grid("bottom_grid.png",    bottom,    row_labels)
        self._render_grid("mean_grid.png",      near_mean, row_labels)
        self._render_grid("p10_grid.png",       near_p10,  row_labels)
        self._render_grid("p90_grid.png",       near_p90,  row_labels)

        # histograms
        save_histogram(
            costs, self.out_dir / "cost_hist.png",
            title="Mask vs ellipse cost",
            xlabel="Pixel difference",
            vlines=[(mean, "--", f"Mean={mean:.0f}"), (p10, ":", f"P10={p10:.0f}"), (p90, ":", f"P90={p90:.0f}")]
        )
        save_histogram(
            times, self.out_dir / "inference_time_hist.png",
            title="Inference time per sample",
            xlabel="ms",
            vlines=[(times.mean(), "--", f"Mean={times.mean():.2f} ms")]
        )

        return {
            "samples": n,
            "mean_cost": mean,
            "p10_cost": p10,
            "p90_cost": p90,
            "mean_inference_ms": float(times.mean()),
        }

    # ------------- analysis ops -------------

    @staticmethod
    def _logits_to_mask(logits: torch.Tensor, thr: float) -> torch.Tensor:
        return (torch.sigmoid(logits) > thr).float()

    @staticmethod
    def _morph_edge(mask_1hw: torch.Tensor) -> np.ndarray:
        m = mask_1hw.squeeze(0).detach().cpu().numpy().astype(np.uint8)
        edge = cv2.morphologyEx(m, cv2.MORPH_GRADIENT, np.ones((3, 3), np.uint8))
        return edge.astype(bool)

    @staticmethod
    def _keep_largest_cc(edge_b: np.ndarray) -> np.ndarray:
        num, labels, stats, _ = cv2.connectedComponentsWithStats(edge_b.astype(np.uint8), connectivity=8)
        if num <= 1:
            return edge_b
        largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
        return labels == largest

    @staticmethod
    def _fit_ellipse(edge_b: np.ndarray) -> Optional[Tuple]:
        ys, xs = np.nonzero(edge_b)
        if xs.size < 5:
            return None
        pts = np.stack([xs, ys], 1).astype(np.float32)[:, None, :]
        try:
            return cv2.fitEllipse(pts)
        except cv2.error:
            return None

    @staticmethod
    def _ellipse_mask(shape: Tuple[int, int], ell: Optional[Tuple]) -> np.ndarray:
        h, w = shape
        m = np.zeros((h, w), np.uint8)
        if ell is None:
            return m
        (cx, cy), (MA, ma), ang = ell
        c = (int(round(cx)), int(round(cy)))
        a = (max(int(MA / 2), 1), max(int(ma / 2), 1))
        cv2.ellipse(m, c, a, ang, 0, 360, 1, thickness=-1, lineType=cv2.LINE_AA)
        return m

    @staticmethod
    def _confusion_rgb(mask_t: torch.Tensor, ellipse_m: np.ndarray) -> np.ndarray:
        """Red = ellipse only; Green = mask only; White = overlap."""
        m = mask_t.squeeze(0).cpu().numpy().astype(bool)
        e = ellipse_m.astype(bool)
        both, m_only, e_only = (m & e), (m & ~e), (~m & e)
        rgb = np.zeros((*m.shape, 3), float)
        rgb[..., 0][e_only] = 1.0
        rgb[..., 1][m_only] = 1.0
        rgb[both] = 1.0
        return rgb

    @staticmethod
    def _overlay_edge(img_chw: torch.Tensor, edge_b: np.ndarray) -> np.ndarray:
        img = img_chw.permute(1, 2, 0).cpu().numpy().copy()
        if img.shape[2] == 1:
            img = np.repeat(img, 3, 2)
        r, g, b = img[..., 0], img[..., 1], img[..., 2]
        r[edge_b] = 1.0; g[edge_b] = 0.0; b[edge_b] = 0.0
        return img

    @staticmethod
    def _overlay_ellipse(img_chw: torch.Tensor, ell: Optional[Tuple]) -> np.ndarray:
        img = img_chw.permute(1, 2, 0).cpu().numpy().copy()
        if img.shape[2] == 1:
            img = np.repeat(img, 3, 2)
        u8 = (img * 255).astype(np.uint8)
        if ell is not None:
            (cx, cy), (MA, ma), ang = ell
            c = (int(round(cx)), int(round(cy)))
            a = (max(int(MA / 2), 1), max(int(ma / 2), 1))
            cv2.ellipse(u8, c, a, ang, 0, 360, (0, 255, 255), 2, cv2.LINE_AA)
        return u8.astype(np.float32) / 255.0

    # ------------- forward + column build -------------

    def _forward(self, x_1chw: torch.Tensor) -> tuple[torch.Tensor, float]:
        if self.device.type == "cuda":
            torch.cuda.synchronize()
        t0 = time.perf_counter()
        logits = self.model(x_1chw)
        if self.device.type == "cuda":
            torch.cuda.synchronize()
        ms = (time.perf_counter() - t0) * 1000.0
        return self._logits_to_mask(logits, self.threshold).cpu().squeeze(0), ms

    def _make_column(self, img: torch.Tensor, mask: torch.Tensor, pred_1hw: torch.Tensor) -> tuple[tuple[np.ndarray, ...], int]:
        edge = self._keep_largest_cc(self._morph_edge(pred_1hw))
        ell = self._fit_ellipse(edge)
        ell_m = self._ellipse_mask(edge.shape, ell)
        col = (
            img.permute(1, 2, 0).cpu().numpy() if img.shape[0] == 3 else np.repeat(img.permute(1, 2, 0).cpu().numpy(), 3, 2),
            ell_m[..., None].astype(np.float32),                 # renamed "Mask"
            self._confusion_rgb(mask, ell_m),
            self._overlay_edge(img, edge),
            self._overlay_ellipse(img, ell),
        )
        # XOR cost (ellipse mask vs mask)
        gt = mask.squeeze(0).cpu().numpy().astype(np.uint8)
        cost = int(np.count_nonzero(gt ^ ell_m.astype(np.uint8)))
        return col, cost

    # ------------- clean grid renderer (table layout) -------------

    @staticmethod
    def _to_rgb_u8(img: np.ndarray) -> np.ndarray:
        """Return HxWx3 uint8 in [0,255]. Accepts (H,W), (H,W,1), float [0,1]/[0,255]."""
        arr = np.asarray(img)
        if arr.ndim == 2:
            arr = arr[..., None]
        if arr.ndim == 3 and arr.shape[2] == 1:
            arr = np.repeat(arr, 3, axis=2)
        if arr.dtype != np.uint8:
            # try normalize floats
            if arr.max() <= 1.0:
                arr = (arr * 255.0).astype(np.uint8)
            else:
                arr = np.clip(arr, 0, 255).astype(np.uint8)
        return arr

    @staticmethod
    def _letterbox_to_square(u8rgb: np.ndarray, size: int) -> np.ndarray:
        """Resize keeping ratio and pad to a square (black)."""
        h, w = u8rgb.shape[:2]
        if h == 0 or w == 0:
            return np.zeros((size, size, 3), np.uint8)
        scale = min(size / h, size / w)
        nh, nw = max(1, int(round(h * scale))), max(1, int(round(w * scale)))
        resized = cv2.resize(u8rgb, (nw, nh), interpolation=cv2.INTER_AREA)
        canvas = np.zeros((size, size, 3), np.uint8)
        y0 = (size - nh) // 2
        x0 = (size - nw) // 2
        canvas[y0:y0+nh, x0:x0+nw] = resized
        return canvas

    @staticmethod
    def _label_strip(label: str, height: int, width: int, font_scale: float = 0.6, thickness: int = 2) -> np.ndarray:
        """Create a white strip (HxW) with black text vertically centered."""
        strip = np.ones((height, width, 3), np.uint8) * 255
        # putText in BGR
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)
        x = 8
        y = height // 2 + th // 2
        cv2.putText(strip, label, (x, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness, cv2.LINE_AA)
        return strip

    def _render_grid(
        self,
        filename: str,
        colset: List[Tuple[np.ndarray, ...]],
        row_labels: List[str],
        tile_size: int = 192,
        label_width: int = 120,
        hgap: int = 24,
        vgap: int = 18,
        margins: Tuple[int, int, int, int] = (18, 18, 18, 18),  # top, right, bottom, left
    ) -> None:
        """
        Compose a single canvas:
            [ left label | tile1 | tile2 | ... ]
        Rows (fixed order): Image, Mask, Difference, Predicted edge, Ellipse overlay
        """
        if len(colset) == 0:
            return

        rows = len(row_labels)           # 5
        K = len(colset)                  # columns of samples
        top, right, bottom, left = margins

        # canvas size
        width  = left + label_width + K * tile_size + (K - 1) * hgap + right
        height = top + rows * tile_size + (rows - 1) * vgap + bottom
        canvas = np.ones((height, width, 3), np.uint8) * 255  # white background

        # fixed mapping of row -> index in each 'col' tuple
        row_indices = [0, 1, 2, 3, 4]  # our columns already came as (Image, Mask, Diff, Edge, Overlay)

        # draw
        y = top
        for r in range(rows):
            # label strip
            strip = self._label_strip(row_labels[r], tile_size, label_width)
            canvas[y:y+tile_size, left:left+label_width] = strip

            # tiles
            x = left + label_width
            for c in range(K):
                panel = colset[c][row_indices[r]]
                u8 = self._to_rgb_u8(panel)
                sq = self._letterbox_to_square(u8, tile_size)
                canvas[y:y+tile_size, x:x+tile_size] = sq
                x += tile_size + hgap

            y += tile_size + vgap

        cv2.imwrite(str(self.out_dir / filename), canvas)
