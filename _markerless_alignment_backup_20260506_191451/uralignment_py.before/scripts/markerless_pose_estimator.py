#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Markerless object-pose node for Robot-Alignment.

This node is intentionally shaped as a drop-in replacement for the C++ AprilTag
pose node. It subscribes to the host-side uralignment_cpp D435i image topic and
publishes only the same cMo TransformStamped interface consumed by ur_e_series
and displays.
"""

from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as SciRot
from sensor_msgs.msg import Image

from flange_pose_estimator.flange_pose_estimator import FlangePoseEstimator


class MarkerlessPoseEstimator(Node):
    """Estimate flange pose from RGB images and publish AprilTag-compatible cMo."""

    def __init__(self) -> None:
        super().__init__('markerless_pose_estimator')

        # Match uralignment_cpp d435i_camera/apriltags topic contract by default.
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('cmo_topic', 'cMo')
        self.declare_parameter('config_filename', 'flange_pose_estimator_config.yaml')
        self.declare_parameter('preferred_encoding', 'bgr8')
        self.declare_parameter('use_active_camera_intrinsics', True)
        self.declare_parameter('log_success_every_n', 30)

        self.bridge = CvBridge()
        self.package_share = Path(get_package_share_directory('uralignment_py'))
        self.config = self._load_estimator_config()
        self._resolve_model_paths(self.config)

        if bool(self.get_parameter('use_active_camera_intrinsics').value):
            self._load_active_camera_intrinsics_into_config(self.config)

        self.estimator = FlangePoseEstimator(self.config)
        self.frame_count = 0
        self.success_count = 0
        self.last_warn_time = 0.0

        image_topic = str(self.get_parameter('image_topic').value)
        cmo_topic = str(self.get_parameter('cmo_topic').value)

        # The C++ camera publishes best_effort/depth=1, so the subscriber must be compatible.
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.cmo_pub = self.create_publisher(TransformStamped, cmo_topic, 10)
        self.image_sub = self.create_subscription(Image, image_topic, self._image_cb, image_qos)

        self.get_logger().info(
            f"markerless_pose_estimator ready: subscribed to {image_topic}, publishing {cmo_topic}"
        )

    # ------------------------------------------------------------------
    # Configuration
    # ------------------------------------------------------------------
    def _load_estimator_config(self) -> dict[str, Any]:
        cfg_name = str(self.get_parameter('config_filename').value)
        cfg_path = self.package_share / 'config' / cfg_name
        if not cfg_path.exists():
            raise FileNotFoundError(f"Estimator config not found: {cfg_path}")
        with cfg_path.open('r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        if not isinstance(cfg, dict):
            raise RuntimeError(f"Estimator config did not parse as a dictionary: {cfg_path}")
        return cfg

    def _resolve_model_paths(self, cfg: dict[str, Any]) -> None:
        """Convert model paths in YAML into absolute package-share paths."""
        models_dir = self.package_share / 'models'
        replacements = (
            ('DETECTION_PARAMS', 'MODEL_PATH', 'detection_model.pt'),
            ('SEGMENTATION_PARAMS', 'MODEL_PATH', 'segmentation_model.pth'),
        )
        for section, key, default_name in replacements:
            sec = cfg.get(section)
            if not isinstance(sec, dict):
                continue
            raw = str(sec.get(key, default_name))
            name = Path(raw).name if raw else default_name
            resolved = models_dir / name
            sec[key] = str(resolved)
            if not resolved.exists():
                self.get_logger().warn(f"Model file not found yet: {resolved}")

    def _candidate_config_dirs(self) -> list[Path]:
        dirs: list[Path] = []

        # Installed package config.
        dirs.append(self.package_share / 'config')

        # Explicit workspace mount used by the Docker run script.
        ws_env = os.environ.get('WS_DIR', '')
        if ws_env:
            ws = Path(ws_env)
            dirs.extend([
                ws / 'config',
                ws / 'config' / 'intrinsics',
                ws / 'src' / 'uralignment_py' / 'config',
                ws / 'src' / 'uralignment_cpp' / 'config',
                ws / 'src' / 'calibration_cpp' / 'config',
                ws / 'install' / 'uralignment_py' / 'share' / 'uralignment_py' / 'config',
                ws / 'install' / 'uralignment_cpp' / 'share' / 'uralignment_cpp' / 'config',
                ws / 'install' / 'calibration_cpp' / 'share' / 'calibration_cpp' / 'config',
            ])

        # Infer workspace root from an installed share path if possible.
        share_str = str(self.package_share)
        marker = '/install/'
        if marker in share_str:
            ws = Path(share_str.split(marker, 1)[0])
            dirs.extend([
                ws / 'config',
                ws / 'config' / 'intrinsics',
                ws / 'src' / 'uralignment_py' / 'config',
                ws / 'src' / 'uralignment_cpp' / 'config',
                ws / 'src' / 'calibration_cpp' / 'config',
            ])

        # If these packages are built in the container, use their share config too.
        for pkg in ('uralignment_cpp', 'calibration_cpp'):
            try:
                dirs.append(Path(get_package_share_directory(pkg)) / 'config')
            except PackageNotFoundError:
                pass

        unique: list[Path] = []
        seen: set[str] = set()
        for d in dirs:
            s = str(d)
            if s not in seen:
                unique.append(d)
                seen.add(s)
        return unique

    @staticmethod
    def _find_first_existing(dirs: list[Path], filename: str) -> Path | None:
        for d in dirs:
            p = d / filename
            if p.exists():
                return p
        return None

    @staticmethod
    def _read_active_camera_yaml(path: Path) -> tuple[str, int, int] | None:
        fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            return None
        try:
            serial = fs.getNode('serial').string()
            width = int(fs.getNode('width').real())
            height = int(fs.getNode('height').real())
        finally:
            fs.release()
        if not serial or width <= 0 or height <= 0:
            return None
        return serial, width, height

    @staticmethod
    def _read_intrinsics_yaml(path: Path) -> tuple[np.ndarray, np.ndarray | None] | None:
        fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            return None
        try:
            k_node = fs.getNode('camera_matrix')
            if k_node.empty():
                k_node = fs.getNode('K')
            K = k_node.mat()

            d_node = fs.getNode('distortion_coefficients')
            if d_node.empty():
                d_node = fs.getNode('D')
            D = None if d_node.empty() else d_node.mat()
        finally:
            fs.release()

        if K is None or K.shape != (3, 3):
            return None
        return K.astype(float), None if D is None else D.astype(float)

    def _load_active_camera_intrinsics_into_config(self, cfg: dict[str, Any]) -> None:
        dirs = self._candidate_config_dirs()
        active_path = self._find_first_existing(dirs, 'active_camera.yaml')
        if active_path is None:
            self.get_logger().warn(
                'active_camera.yaml not found; using CAMERA_PARAMETERS from flange_pose_estimator_config.yaml'
            )
            return

        active = self._read_active_camera_yaml(active_path)
        if active is None:
            self.get_logger().warn(f'Could not parse active_camera.yaml: {active_path}')
            return

        serial, width, height = active
        intr_name = f'{serial}_{width}x{height}_intrinsics.yaml'
        intr_path = self._find_first_existing(dirs, intr_name)
        if intr_path is None:
            self.get_logger().warn(
                f'Could not find {intr_name}; using CAMERA_PARAMETERS from flange_pose_estimator_config.yaml'
            )
            return

        intr = self._read_intrinsics_yaml(intr_path)
        if intr is None:
            self.get_logger().warn(f'Could not parse intrinsics YAML: {intr_path}')
            return

        K, D = intr
        cam = cfg.setdefault('CAMERA_PARAMETERS', {})
        cam['FX'] = float(K[0, 0])
        cam['FY'] = float(K[1, 1])
        cam['UX'] = float(K[0, 2])
        cam['UY'] = float(K[1, 2])
        cam['RESOLUTION'] = [int(width), int(height)]
        if D is not None:
            cam['DISTORTION_COEFFS'] = [float(x) for x in D.reshape(-1)]

        self.get_logger().info(
            f'Loaded active camera intrinsics: serial={serial}, resolution={width}x{height}, file={intr_path}'
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _image_cb(self, msg: Image) -> None:
        self.frame_count += 1
        t0 = time.perf_counter()

        bgr = self._to_cv_bgr(msg)
        if bgr is None:
            return

        ok, T = self._estimate_pose(bgr)
        if not ok or T is None:
            self._warn_throttled('Markerless pose estimation failed')
            return

        self._publish_cmo(msg.header, T)

        self.success_count += 1
        n = int(self.get_parameter('log_success_every_n').value)
        if n > 0 and self.success_count % n == 0:
            dt = time.perf_counter() - t0
            hz = 1.0 / dt if dt > 0.0 else 0.0
            t = np.asarray(T.t, dtype=float).reshape(3)
            rvec = SciRot.from_matrix(np.asarray(T.R, dtype=float)).as_rotvec()
            self.get_logger().info(
                'cMo markerless pose: '
                f't=[{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}] m, '
                f'r=[{rvec[0]:.6f}, {rvec[1]:.6f}, {rvec[2]:.6f}] rad, '
                f'rate={hz:.2f} Hz'
            )

    def _to_cv_bgr(self, msg: Image) -> np.ndarray | None:
        enc = str(self.get_parameter('preferred_encoding').value)
        try:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding=enc)
        except Exception as exc:
            self._warn_throttled(f'cv_bridge conversion failed: {exc}')
            return None

    def _estimate_pose(self, bgr: np.ndarray) -> tuple[bool, Any | None]:
        try:
            result = self.estimator.get_pose(bgr)
        except Exception as exc:
            self._warn_throttled(f'Estimator exception: {exc}')
            return False, None

        if result is None:
            return False, None
        if not isinstance(result, tuple) or len(result) != 2:
            return False, None

        ok, T = result
        return bool(ok), T

    def _publish_cmo(self, header: Any, T: Any) -> None:
        t = np.asarray(T.t, dtype=float).reshape(3)
        Rm = np.asarray(T.R, dtype=float)
        q = SciRot.from_matrix(Rm).as_quat()  # [x, y, z, w]

        msg = TransformStamped()
        msg.header.stamp = header.stamp
        msg.header.frame_id = 'camera_frame'
        msg.child_frame_id = 'apriltag'
        msg.transform.translation.x = float(t[0])
        msg.transform.translation.y = float(t[1])
        msg.transform.translation.z = float(t[2])
        msg.transform.rotation.x = float(q[0])
        msg.transform.rotation.y = float(q[1])
        msg.transform.rotation.z = float(q[2])
        msg.transform.rotation.w = float(q[3])
        self.cmo_pub.publish(msg)

    def _warn_throttled(self, text: str, period_s: float = 2.0) -> None:
        now = time.monotonic()
        if now - self.last_warn_time >= period_s:
            self.get_logger().warn(text)
            self.last_warn_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MarkerlessPoseEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
