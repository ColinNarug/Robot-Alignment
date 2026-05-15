#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations
import numpy as np
import cv2
from collections import deque
from typing import Dict, Tuple, Optional

"""
Class that draws one or more time-series as colored lines on an OpenCV canvas with grid and labels.
Used for visualizing real-time numeric data (e.g., pose errors, angles, etc.).
"""

class Plotter:
    
	def __init__(self, history_len: int = 200):
		self.N = int(history_len)                         # data history length per series
		self.data: Dict[str, deque] = {}                  # label -> deque of float samples
		self.units: Dict[str, str] = {}                   # label -> physical unit (e.g., 'deg', 'mm')
		self.colors: Dict[str, Tuple[int, int, int]] = {} # label -> BGR color for drawing
		self._canvas: Optional[np.ndarray] = None         # optional drawing buffer (H x W x 3)

	# ---------- setup ----------
	def add_series(self, label: str, unit: str, color: Tuple[int, int, int]) -> None:
		self.data[label] = deque(maxlen=self.N)
		self.units[label] = unit
		self.colors[label] = color

	# ---------- update ----------
	def append_sample(self, values: Dict[str, float]) -> None:
		for k, v in values.items():
			if k in self.data:
				self.data[k].append(float(v))

	# ---------- buffer ----------
	def allocate_panel(self, view_h: int, view_w: int) -> np.ndarray:
		h = max(200, int(0.7 * view_h))  # panel height based on view size
		w = max(240, int(0.5 * view_w))  # panel width based on view size
		return np.zeros((h, w, 3), np.uint8)  # black canvas

	def set_canvas(self, canvas: np.ndarray) -> None:
		if canvas is None or canvas.size == 0:
			raise ValueError("Invalid canvas.")
		self._canvas = canvas  # store canvas internally

	def get_canvas(self, copy: bool = False) -> np.ndarray:
		if self._canvas is None:
			raise RuntimeError("Canvas not set. Use set_canvas() or pass one to render().")
		return self._canvas.copy() if copy else self._canvas

	# ---------- internals ----------
	def _nice_step(self, max_abs: float) -> float:
		if max_abs <= 1e-12:
			return 1.0
		exp = int(np.floor(np.log10(max_abs)))
		base = 10 ** exp
		for m in [1, 2, 5, 10]:
			if max_abs / (m * base) <= 5:
				return m * base
		return base

	# ---------- rendering ----------
	def render(self, canvas: Optional[np.ndarray] = None, title: str = "") -> None:
		"""
		Draws time-series data.
		Handles empty buffers safely (shows 'waiting for data...').
		"""
		dst = canvas if canvas is not None else self._canvas
		if dst is None:
			raise RuntimeError("No canvas to draw on. Pass a canvas or call set_canvas().")

		H, W = dst.shape[:2]        # panel height and width
		dst[:] = (255, 255, 255)    # white background

		# if no data yet
		if not self.data or all(len(d) == 0 for d in self.data.values()):
			cv2.putText(dst, "waiting for data...", (20, H // 2),
						cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 1)
			return

		# flatten all numeric values for scaling
		try:
			all_vals = np.concatenate([np.asarray(d, float)
									   for d in self.data.values() if len(d) > 0], axis=0)
		except Exception:
			cv2.putText(dst, "invalid data", (20, H // 2),
						cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 1)
			return

		max_abs = max(1e-6, float(np.max(np.abs(all_vals))))  # global value range
		step = self._nice_step(max_abs)                      # adaptive grid spacing
		y_max = max(step, np.ceil(max_abs / step) * step)
		y_min = -y_max

		L, R = 60, W - 20         # left/right margins
		T, B = 50, H - 40         # top/bottom margins
		width, height = R - L, B - T
		font = cv2.FONT_HERSHEY_SIMPLEX
		col_grid = (220, 220, 220)  # grid color

		def y_to_pix(y: float) -> int:
			return int(B - (y - y_min) / (y_max - y_min) * height)

		# horizontal grid + y labels
		for y in np.arange(y_min, y_max + step, step):
			yp = y_to_pix(y)
			cv2.line(dst, (L, yp), (R, yp), col_grid, 1)
			label = "0" if abs(y) < 1e-10 else f"{y:.3g}"
			cv2.putText(dst, label, (8, yp + 4), font, 0.4, (0, 0, 0), 1)

		# vertical grid
		for i in range(11):
			x = L + int(i * width / 10)
			cv2.line(dst, (x, T), (x, B), col_grid, 1)

		# draw data series
		for label, d in self.data.items():
			if len(d) < 2:
				continue
			xs = np.linspace(L, R, len(d)).astype(int)
			ys = np.vectorize(y_to_pix)(np.asarray(d, float))
			pts = np.stack([xs, ys], axis=1).reshape(-1, 1, 2)
			cv2.polylines(dst, [pts], False, self.colors[label], 2)

		# titles and labels
		unit = next(iter(self.units.values()), "")
		cv2.putText(dst, f"[{unit}]", (8, T - 14), font, 0.5, (0, 0, 0), 1)
		cv2.putText(dst, title, ((L + R) // 2 - 60, T - 14), font, 0.6, (60, 60, 60), 1)
		for i, label in enumerate(self.data.keys()):
			cv2.putText(dst, label, (R - 150, T + 16 * (i + 1)),
						font, 0.45, self.colors[label], 1)
