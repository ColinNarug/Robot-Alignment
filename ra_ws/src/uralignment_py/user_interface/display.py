#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations
import numpy as np
import cv2
from spatialmath import SE3


"""
Class that draws SE3 poses as RGB axes projected on an image.
- Current pose: RGB arrows.
- Desired pose: semi-transparent RGB arrows.
"""

class Display:

    def __init__(self, K: np.ndarray, axis_len_m: float = 0.03):
        # Camera intrinsics (3x3) and axis length in meters
        self.K = np.asarray(K, dtype=np.float32)
        self.axis_len = float(axis_len_m)
        self.frame: np.ndarray | None = None

    # ----------------- frame I/O -----------------
    def set_frame(self, img_bgr: np.ndarray) -> None:
        """Set the target image buffer."""
        if img_bgr is None or img_bgr.size == 0:
            raise ValueError("Invalid frame: empty or None.")
        self.frame = img_bgr.copy()

    def get_frame(self, copy: bool = False) -> np.ndarray:
        """Get the current image buffer (optionally as a copy)."""
        if self.frame is None:
            raise RuntimeError("Frame not initialized. Call set_frame() first.")
        return self.frame.copy() if copy else self.frame

    # ----------------- helpers -----------------
    def _draw_axis(self, img: np.ndarray, start_xy: np.ndarray, end_xy: np.ndarray,
                   color: tuple[int, int, int], thickness: int = 2) -> None:
        """Draw an arrow from start_xy to end_xy."""
        p1 = (int(start_xy[0]), int(start_xy[1]))
        p2 = (int(end_xy[0]), int(end_xy[1]))
        cv2.arrowedLine(img, p1, p2, color, thickness, cv2.LINE_AA, tipLength=0.25)

    def _project_points(self, pts_cam: np.ndarray) -> np.ndarray:
        """
        Project 3D camera-frame points to 2D pixel coordinates using intrinsics K.
        pts_cam: (N,3) array in camera coordinates.
        Returns: (N,2) array of (u, v) pixel coordinates.
        """
        rvec = np.zeros((3, 1), dtype=np.float32)
        tvec = np.zeros((3, 1), dtype=np.float32)
        pts = pts_cam.reshape(-1, 1, 3).astype(np.float32)
        uv, _ = cv2.projectPoints(pts, rvec, tvec, self.K, None)
        return uv.reshape(-1, 2)

    # ----------------- main API -----------------

    def draw_pose(self, T_cam_obj: SE3 | None, alpha: float = 1.0) -> None:
        """
        Draw an SE3 pose as RGB axes. Safely no-ops on None.
        T_cam_obj must be camera_T_object (object in camera frame).
        """
        if self.frame is None or T_cam_obj is None:
            return

        try:
            # Axis endpoints in the object frame (meters)
            axis_len = self.axis_len
            axes_obj = np.array([
                [0,      0,      0],        # origin
                [axis_len, 0,      0],        # +X
                [0,      axis_len, 0],        # +Y
                [0,      0,      axis_len],   # +Z
            ], dtype=np.float32)

            # Transform to camera frame: Pc = R * Po + t
            R_cam_obj = np.asarray(T_cam_obj.R, dtype=np.float32)
            t_cam_obj = np.asarray(T_cam_obj.t, dtype=np.float32).reshape(1, 3)
            axes_cam = (R_cam_obj @ axes_obj.T).T + t_cam_obj  # shape (4,3)

            # Project to pixels
            uv_px = self._project_points(axes_cam)  # shape (4,2)
            origin_px, x_px, y_px, z_px = uv_px

            # Choose drawing target 
            dst = self.frame if alpha >= 1.0 else self.frame.copy()

            # Draw axes: X=red, Y=green, Z=blue
            self._draw_axis(dst, origin_px, x_px, (0, 0, 255), thickness=2)
            self._draw_axis(dst, origin_px, y_px, (0, 255, 0), thickness=2)
            self._draw_axis(dst, origin_px, z_px, (255, 0, 0), thickness=2)

            # Alpha blend (semi-transparent overlay)
            if alpha < 1.0:
                cv2.addWeighted(dst, alpha, self.frame, 1.0 - alpha, 0.0, dst=self.frame)

        except Exception:
            # Silent skip to avoid crashing the UI loop
            return
