#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Make loss plots (no CSV):
- loss_curve.png
- val_scatter.png
- overfit_gap.png
"""

from pathlib import Path
from typing import Sequence
import numpy as np
import matplotlib
matplotlib.use("Agg") 
import matplotlib.pyplot as plt


def _moving_average(x: np.ndarray, k: int) -> np.ndarray:
    """
    Apply centered moving average with window size k (must be odd).
    Pads input with edge values to preserve length.
    """
    if k <= 1 or k % 2 == 0 or len(x) < k:
        return x.copy()
    pad = k // 2
    xpad = np.pad(x, (pad, pad), mode="edge")
    return np.convolve(xpad, np.ones(k) / k, mode="valid")


def _save_loss_curve(out_path: Path, epochs: np.ndarray, tr: np.ndarray, val: np.ndarray, smooth_window: int) -> None:
    """
    Save line plot of training and validation loss across epochs.
    Optionally overlays moving average curves for smoothing.
    """
    fig = plt.figure(figsize=(8, 5))
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(epochs, tr, marker="o", linewidth=1, label="Train")
    ax.plot(epochs, val, marker="o", linewidth=1, label="Val")
    if smooth_window and smooth_window > 1:
        ax.plot(epochs, _moving_average(tr, smooth_window),  linestyle="--", linewidth=2, label=f"Train (MA{smooth_window})")
        ax.plot(epochs, _moving_average(val, smooth_window), linestyle="--", linewidth=2, label=f"Val (MA{smooth_window})")
    ax.set_xlabel("Epoch"); ax.set_ylabel("Loss"); ax.set_title("Training & Validation Loss")
    ax.grid(True, alpha=0.3); ax.legend(); fig.tight_layout(); fig.savefig(out_path, dpi=150); plt.close(fig)


def _save_val_scatter(out_path: Path, epochs: np.ndarray, val: np.ndarray) -> None:
    """
    Save scatter plot of validation loss by epoch.
    Highlights the epoch with minimum validation loss using a star marker.
    """
    best_idx = int(np.argmin(val))
    fig = plt.figure(figsize=(7, 5))
    ax = fig.add_subplot(1, 1, 1)
    ax.scatter(epochs, val, s=30)
    ax.scatter([epochs[best_idx]], [val[best_idx]], s=80, marker="*", zorder=3)
    ax.set_xlabel("Epoch"); ax.set_ylabel("Val Loss"); ax.set_title("Validation Loss by Epoch (★ = best)")
    ax.grid(True, alpha=0.3); fig.tight_layout(); fig.savefig(out_path, dpi=150); plt.close(fig)


def _save_overfit_gap(out_path: Path, epochs: np.ndarray, tr: np.ndarray, val: np.ndarray) -> None:
    """
    Save line plot of the overfitting gap: (val_loss - train_loss).
    A horizontal line at 0 is drawn for reference.
    """
    gap = val - tr
    fig = plt.figure(figsize=(8, 4))
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(epochs, gap, marker="o")
    ax.axhline(0.0, linewidth=1)
    ax.set_xlabel("Epoch"); ax.set_ylabel("Val - Train"); ax.set_title("Overfitting Gap")
    ax.grid(True, alpha=0.3); fig.tight_layout(); fig.savefig(out_path, dpi=150); plt.close(fig)


def main(
    train_losses: Sequence[float],
    val_losses:   Sequence[float],
    out_dir: str | Path = "./results",
    smooth_window: int = 5,
) -> None:
    """
    Generate loss plots from training and validation loss sequences.
    Saves: loss_curve.png, val_scatter.png, overfit_gap.png in out_dir.
    """
    tr = np.asarray(train_losses, dtype=float)
    va = np.asarray(val_losses,   dtype=float)
    assert tr.ndim == 1 and va.ndim == 1 and len(tr) == len(va) and len(tr) > 0
    epochs = np.arange(len(tr), dtype=int)
    out = Path(out_dir); out.mkdir(parents=True, exist_ok=True)
    _save_loss_curve(out / "loss_curve.png", epochs, tr, va, smooth_window)
    _save_val_scatter(out / "val_scatter.png", epochs, va)
    _save_overfit_gap(out / "overfit_gap.png", epochs, tr, va)
