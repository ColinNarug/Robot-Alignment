#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
COCO JSON -> PNG ellipse masks

Overview
--------
This program converts a COCO-style annotation file into binary mask images
(one per COCO image). Each mask contains ONLY the largest ellipse fitted over
the union of all instance masks in that image.

Fixed relative paths
--------------------
- Input JSON directory:  ./json_labels/
- Output mask directory: ./mask_labels/  
"""

from __future__ import annotations
import math
from pathlib import Path
import yaml
import numpy as np
from PIL import Image
import cv2
from pycocotools.coco import COCO


# ================================================================
class COCOEllipseMaskGenerator:
	"""Convert COCO JSON segmentations into ellipse-based binary masks."""

	def __init__(self, config_path: Path):
		"""
		Load YAML configuration and prepare paths.
		- Input dir is rigid:  ./json_labels/
		- Output dir is in (./mask_labels...) 
		"""
		self.script_dir = config_path.parent
		self.json_dir   = self.script_dir / "json_labels"
		base_mask_dir   = self.script_dir / "mask_labels"
		self.mask_dir   = self._resolve_unique_output_dir(base_mask_dir)

		# Load YAML config
		if not config_path.exists():
			raise FileNotFoundError(f"Config file not found: {config_path}")
		with open(config_path, "r", encoding="utf-8") as f:
			cfg = yaml.safe_load(f)

		self.json_filename = cfg.get("json_filename", None)
		if not self.json_filename:
			raise ValueError("Missing 'json_filename' in config.yaml")

		self.coco_json = self.json_dir / self.json_filename
		self._ensure_dir(self.mask_dir)

		if not self.coco_json.exists():
			raise FileNotFoundError(f"COCO JSON not found: {self.coco_json}")

		# Load COCO annotations
		self.coco = COCO(str(self.coco_json))

	# -------------------------
	# Internal utilities
	# -------------------------

	def _ensure_dir(self, path: Path) -> None:
		"""Create directory if missing."""
		path.mkdir(parents=True, exist_ok=True)

	def _resolve_unique_output_dir(self, base_dir: Path) -> Path:
		"""
		Return a directory path that is unique:
		- If base_dir doesn't exist, return base_dir.
		- Otherwise, return base_dir_<k> with the smallest k >= 1 that doesn't exist.
		Note: directory is NOT created here; caller decides when to create it.
		"""
		if not base_dir.exists():
			return base_dir
		k = 1
		while True:
			candidate = base_dir.parent / f"{base_dir.name}_{k}"
			if not candidate.exists():
				return candidate
			k += 1

	def _ann_to_mask(self, ann: dict, height: int, width: int) -> np.ndarray:
		"""Convert one COCO annotation to a {0,1} uint8 mask."""
		m = self.coco.annToMask(ann)
		if m.shape != (height, width):
			m = cv2.resize(m.astype(np.uint8), (width, height), interpolation=cv2.INTER_NEAREST)
		return (m > 0).astype(np.uint8)

	def _compose_union_mask(self, image_id: int, height: int, width: int) -> np.ndarray:
		"""Return the union of all instance masks for the given image."""
		ann_ids = self.coco.getAnnIds(imgIds=[image_id])
		anns = self.coco.loadAnns(ann_ids)
		if not anns:
			return np.zeros((height, width), dtype=np.uint8)
		acc = np.zeros((height, width), dtype=np.uint8)
		for ann in anns:
			acc = np.maximum(acc, self._ann_to_mask(ann, height, width))
		return (acc * 255).astype(np.uint8)

	def _mask_to_largest_filled_ellipse(self, mask_0_255: np.ndarray) -> np.ndarray:
		"""
		Fit ellipse(s) on mask contours and return a mask with only the largest one filled.
		If fitting fails, return the input mask unchanged.
		"""
		bin_mask = (mask_0_255 > 0).astype(np.uint8)
		contours, _ = cv2.findContours(bin_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

		best = None
		best_area = 0.0
		for cnt in contours:
			if len(cnt) < 5:
				continue
			try:
				(cx, cy), (MA, ma), ang = cv2.fitEllipse(cnt)
				if MA <= 0 or ma <= 0:
					continue
				area = math.pi * 0.5 * MA * 0.5 * ma  # axes are full lengths
				if area > best_area:
					best_area = area
					best = ((cx, cy), (MA, ma), ang)
			except cv2.error:
				continue

		if best is None:
			return mask_0_255

		h, w = bin_mask.shape[:2]
		out = np.zeros((h, w), dtype=np.uint8)
		cv2.ellipse(out, best, color=255, thickness=-1)
		return out

	# -------------------------
	# main process
	# -------------------------
	def run(self) -> None:
		"""Process all COCO images and write ellipse masks."""
		img_ids = self.coco.getImgIds()
		imgs = self.coco.loadImgs(img_ids)
		print(f"[INFO] Loaded {len(imgs)} COCO images from {self.coco_json}")
		print(f"[INFO] Output directory: {self.mask_dir}")

		written = 0
		for im in imgs:
			file_name = im["file_name"]
			h, w = int(im["height"]), int(im["width"])
			union = self._compose_union_mask(im["id"], h, w)
			ellipse_mask = self._mask_to_largest_filled_ellipse(union)
			out_path = self.mask_dir / f"{Path(file_name).stem}.png"
			Image.fromarray(ellipse_mask, mode="L").save(out_path)
			written += 1
			if written % 50 == 0:
				print(f"[INFO] Written {written} masks...")

		print(f"[DONE] Wrote {written} mask(s) to {self.mask_dir}")


def main() -> None:
	"""Load config.yaml and run the COCO ellipse mask generator."""
	script_dir = Path(__file__).resolve().parent
	config_path = script_dir / "config.yaml"
	gen = COCOEllipseMaskGenerator(config_path)
	gen.run()


if __name__ == "__main__":
	main()
