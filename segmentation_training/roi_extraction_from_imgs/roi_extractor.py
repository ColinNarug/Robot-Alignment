#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Square ROI extractor from YOLO detections (single class).

Overview
--------
This program scans ./imgs/ for input images and writes per-detection square ROIs
to ./roi_extracted/. It runs an Ultralytics YOLO model (specified by name in
config.yaml, located next to this script).

Fixed relative paths (do not change)
------------------------------------
- Input images folder: ./imgs/
- Output ROI folder:   ./roi_extracted/

Configuration (the only external parameter)
-------------------------------------------
- A YAML file named `config.yaml` must sit next to this script and contain:
      model_filename: "best.pt"

Behavior
--------
- Recursively collects *.png / *.PNG under ./imgs/.
- Runs YOLO inference on each image.
- Keeps detections whose confidence >= CONF_THRESHOLD and whose class == CLASS_ID.
- Expands each bbox by EXPAND_FACTOR per side, converts it to a square window,
  clamps to image bounds, and writes the ROI as PNG in ./roi_extracted/.

Notes
-----
- Dependencies: ultralytics, opencv-python, numpy, pyyaml, pillow
- You can adjust CLASS_ID / CONF_THRESHOLD / EXPAND_FACTOR below if needed.
"""

from __future__ import annotations
from pathlib import Path
from typing import List

import yaml
import cv2
import numpy as np
from ultralytics import YOLO


class YOLOSquareROIExtractor:
	"""Extract square ROIs from YOLO detections using fixed project folders."""

	# --- Tunables (kept in code, not in YAML as requested) ---
	CLASS_ID: int = 0          # single target class id to keep
	CONF_THRESHOLD: float = 0.7
	EXPAND_FACTOR: float = 0.15  # expand bbox by 20% per side before squaring

	def __init__(self, config_path: Path):
		"""
		Read config.yaml, resolve fixed folders, and load the YOLO model.

		Parameters
		----------
		config_path : Path
			Path to the YAML file next to this script. Must contain:
			    model_filename: "<weights.pt>"
		"""
		self.script_dir = config_path.parent
		self.imgs_dir = self.script_dir / "imgs"
		self.roi_dir  = self.script_dir / "roi_extracted"

		# Load YAML (only model filename)
		if not config_path.exists():
			raise FileNotFoundError(f"Config file not found: {config_path}")
		with open(config_path, "r", encoding="utf-8") as f:
			cfg = yaml.safe_load(f) or {}

		model_filename = cfg.get("model_filename")
		if not model_filename:
			raise ValueError("Missing 'model_filename' in config.yaml")
		self.weights_path = (self.script_dir / model_filename).resolve()

		if not self.weights_path.exists():
			raise FileNotFoundError(f"Model weights not found: {self.weights_path}")

		# Prepare folders
		self._ensure_dir(self.roi_dir)

		# Load model once
		self.model = YOLO(str(self.weights_path))

	# --------------- helpers ---------------

	@staticmethod
	def _ensure_dir(path: Path) -> None:
		path.mkdir(parents=True, exist_ok=True)

	@staticmethod
	def _gather_png_recursive(root: Path) -> List[Path]:
		"""Return sorted list of *.png/PNG found recursively under root."""
		paths: List[Path] = []
		for pat in ("*.png", "*.PNG"):
			paths.extend(root.rglob(pat))
		return sorted(paths)

	@staticmethod
	def _expand_square_and_clamp(xyxy: np.ndarray, w: int, h: int, factor: float) -> np.ndarray:
		"""
		Expand [x1,y1,x2,y2] by factor per side, make it square (max side), clamp to image bounds.
		Returns int array [x1, y1, x2, y2].
		"""
		x1, y1, x2, y2 = xyxy.astype(float)

		# proportional expansion
		bw, bh = x2 - x1, y2 - y1
		dx, dy = bw * factor, bh * factor
		x1 -= dx; y1 -= dy
		x2 += dx; y2 += dy

		# square side
		bw, bh = x2 - x1, y2 - y1
		side = max(1.0, min(max(bw, bh), w - 1, h - 1))

		# center square
		cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
		half = side / 2.0
		x1 = cx - half; x2 = cx + half
		y1 = cy - half; y2 = cy + half

		# clamp to bounds
		if x1 < 0:   x2 -= x1; x1 = 0
		if x2 > w-1: shift = x2 - (w - 1); x1 -= shift; x2 = w - 1
		if y1 < 0:   y2 -= y1; y1 = 0
		if y2 > h-1: shift = y2 - (h - 1); y1 -= shift; y2 = h - 1

		# convert to int and guarantee positive area
		x1 = int(max(0, round(x1))); y1 = int(max(0, round(y1)))
		x2 = int(min(w - 1, round(x2))); y2 = int(min(h - 1, round(y2)))
		if x2 <= x1: x2 = min(x1 + 1, w - 1)
		if y2 <= y1: y2 = min(y1 + 1, h - 1)
		return np.array([x1, y1, x2, y2], dtype=int)

	# --------------- main ---------------

	def run(self) -> None:
		"""Process all images under ./imgs/ and write ROIs to ./roi_extracted/."""
		if not self.imgs_dir.exists():
			raise FileNotFoundError(f"Input folder not found: {self.imgs_dir}")

		image_paths = self._gather_png_recursive(self.imgs_dir)
		print(f"[INFO] Model: {self.weights_path.name}")
		print(f"[INFO] Images: {len(image_paths)} found under {self.imgs_dir}")
		print(f"[INFO] Output: {self.roi_dir}")

		roi_counter = 1
		for img_path in image_paths:
			img = cv2.imread(str(img_path), cv2.IMREAD_COLOR)
			if img is None:
				print(f"[WARN] Cannot read: {img_path}")
				continue
			h, w = img.shape[:2]

			# Run YOLO
			results = self.model.predict(source=img, conf=self.CONF_THRESHOLD, verbose=False)
			if not results or results[0].boxes is None or len(results[0].boxes) == 0:
				print(f"[INFO] {img_path.name}: no detections")
				continue

			boxes = results[0].boxes
			cls   = boxes.cls.cpu().numpy() if boxes.cls is not None else np.empty((0,))
			xyxy  = boxes.xyxy.cpu().numpy() if boxes.xyxy is not None else np.empty((0, 4))
			conf  = boxes.conf.cpu().numpy() if boxes.conf is not None else np.empty((0,))

			# Filter detections: single class + confidence threshold
			keep_idx = [
				i for i, c in enumerate(cls)
				if int(c) == self.CLASS_ID and (conf[i] if conf.size else 1.0) >= self.CONF_THRESHOLD
			]

			saved_here = 0
			for i in keep_idx:
				x1, y1, x2, y2 = self._expand_square_and_clamp(xyxy[i], w, h, self.EXPAND_FACTOR)
				roi = img[y1:y2, x1:x2]
				if roi.size == 0:
					continue
				out = self.roi_dir / f"{img_path.stem}_roi{roi_counter}.png"
				cv2.imwrite(str(out), roi)
				roi_counter += 1
				saved_here += 1

			print(f"[OK] {img_path.name}: saved {saved_here} ROI(s)")

		print(f"[DONE] Total ROIs: {roi_counter - 1}  → {self.roi_dir}")


def main() -> None:
	"""Load config.yaml (next to this script) and run the extractor."""
	script_dir = Path(__file__).resolve().parent
	config_path = script_dir / "config.yaml"
	extractor = YOLOSquareROIExtractor(config_path)
	extractor.run()


if __name__ == "__main__":
	main()
