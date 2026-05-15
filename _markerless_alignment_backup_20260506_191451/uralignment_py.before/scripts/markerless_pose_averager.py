#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from collections import deque
import numpy as np
import yaml
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R
from scipy.linalg import eigh
from ament_index_python.packages import get_package_share_directory

from ur_alignment.flange_pose_estimator import FlangePoseEstimator


"""
This node subscribes to /realsense/image_raw, estimates the flange pose for each frame,
computes the average SE(3) over a short buffer, and publishes /image_with_pose_avg
showing both current and averaged poses as RGB axes.
"""


# ---------- Quaternion / Pose averaging helpers ----------
def average_quaternions(quats, weights=None):
    """Compute the weighted average quaternion from a set of quaternions."""
    if weights is not None and len(quats) != len(weights):
        raise ValueError("Lengths of quaternions and weights must match.")
    accum = np.zeros((4, 4))
    for i, quat in enumerate(quats):
        w = weights[i] if weights is not None else 1.0
        q = np.array(quat, dtype=float)
        accum += w * np.outer(q, q)
    vals, vecs = eigh(accum)
    q_avg = vecs[:, np.argmax(vals)]
    return q_avg / np.linalg.norm(q_avg)


def average_T(T_list):
    """Compute average transform: rotation (deg std) and translation (per-axis std)."""
    quats, translations = [], []
    for T in T_list:
        Rm = R.from_matrix(T[:3, :3])
        quats.append(Rm.as_quat())   # [x y z w]
        translations.append(T[:3, 3])

    translations = np.asarray(translations, dtype=float)
    avg_t = translations.mean(axis=0)
    std_t = translations.std(axis=0)

    q_avg = average_quaternions(quats)
    avg_R = R.from_quat(q_avg).as_matrix()

    angles = []
    R_avg = R.from_matrix(avg_R)
    for T in T_list:
        R_i = R.from_matrix(T[:3, :3])
        rel = R_avg.inv() * R_i
        angles.append(rel.magnitude())
    std_angle_deg = np.degrees(np.std(angles))
    return avg_R, std_angle_deg, avg_t, std_t


# ---------- Node ----------
class MarkerlessPoseAverager(Node):

    def __init__(self) -> None:
        super().__init__('markerless_pose_averager')

        self._declare_params()
        self.cfg = self._load_config()

        self.bridge = CvBridge()
        self.estimator = FlangePoseEstimator(self.cfg)

        # Parameters
        self.window_size = int(self.get_parameter('window_size').get_parameter_value().integer_value)
        self.publish_debug_image = True 
        self.image_topic = '/realsense/image_raw'

        # Pose buffer
        self.pose_buffer = deque(maxlen=self.window_size)

        # Publishers
        self.success_pub = self.create_publisher(Bool, '/pose_estimation_success', 10)
        self.debug_img_pub = self.create_publisher(Image, '/image_with_pose_avg', 10)

        qos_image = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub_image = self.create_subscription(Image, self.image_topic, self._on_image, qos_image)

        # Intrinsics (mandatory from config)
        self._load_intrinsics_from_config()

    # ---------- Params / config ----------
    def _declare_params(self):
        """Declare parameters"""
        self.declare_parameter('window_size', 5)

    def _load_config(self) -> dict:
        """Load estimator configuration file from package share."""
        pkg = get_package_share_directory('ur_alignment')
        cfg_path = os.path.join(pkg, 'config', 'flange_pose_estimator_config.yaml')
        with open(cfg_path, 'r') as f: 
            return yaml.safe_load(f)

    def _load_intrinsics_from_config(self):
        """Load camera intrinsics from configuration file."""
        cam_params = self.cfg.get('CAMERA_PARAMETERS', None)
        self.fx = float(cam_params['FX'])
        self.fy = float(cam_params['FY'])
        self.cx = float(cam_params['UX'])
        self.cy = float(cam_params['UY'])

    # ---------- Image callback ----------
    def _on_image(self, msg: Image) -> None:
        """Process incoming image, estimate pose, update average, and publish debug image."""
        bgr = self._to_cv_bgr(msg)
        if bgr is None: return

        ok, T_obj = self._estimate_pose(bgr)
        if not ok or T_obj is None: return

        T44 = self._to_homogeneous(T_obj.R, T_obj.t)
        self.pose_buffer.append(T44)

        avg_R, _, avg_t, _ = average_T(list(self.pose_buffer))

        # Print blocks
        self._print_pose_block("AVERAGE POSE", avg_R, avg_t)

        # Draw annotated image
        annotated = self._draw_axes_overlay(
            bgr,
            T_curr=T44,
            T_avg=self._compose_T(avg_R, avg_t),
            len_avg=0.05,  # m
            len_cur=0.03,  # m
        )
        out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        out_msg.header = msg.header
        self.debug_img_pub.publish(out_msg)

    # ---------- Helpers ----------
    def _to_cv_bgr(self, msg: Image):
        """Convert ROS Image to OpenCV BGR image."""
        try:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image convert error: {e}")
            return None

    def _estimate_pose(self, bgr) -> tuple[bool, object]:
        """Estimate flange pose from an OpenCV image."""
        try:
            ok, T = self.estimator.get_pose(bgr)
            return bool(ok), T
        except Exception as e:
            self.get_logger().error(f"Pose estimation error: {e}")
            return False, None

    @staticmethod
    def _to_homogeneous(Rm: np.ndarray, t: np.ndarray) -> np.ndarray:
        """Build homogeneous transform matrix from rotation and translation."""
        T = np.eye(4, dtype=float)
        T[:3, :3] = np.asarray(Rm, dtype=float)
        T[:3, 3] = np.asarray(t, dtype=float).reshape(3)
        return T

    @staticmethod
    def _compose_T(Rm: np.ndarray, t: np.ndarray) -> np.ndarray:
        """Compose SE(3) homogeneous matrix from R and t."""
        T = np.eye(4, dtype=float)
        T[:3, :3] = Rm
        T[:3, 3] = t.reshape(3)
        return T

    def _print_pose_block(self, title: str, Rm: np.ndarray, t: np.ndarray) -> None:
        """Log a formatted pose block with translation and rotation vector."""
        rvec = R.from_matrix(Rm).as_rotvec()
        lines = [
            f"{title}:",
            "t : [{:.6f}, {:.6f}, {:.6f}]".format(*t),
            "r : [{:.6f}, {:.6f}, {:.6f}]".format(*rvec),
            "#############################",
        ]
        self.get_logger().info("\n".join(lines))

    # ----- Axis overlay -----
    def _draw_axes_overlay(self, img_bgr: np.ndarray, T_curr: np.ndarray, T_avg: np.ndarray,
                           len_avg: float, len_cur: float) -> np.ndarray:
        """Draw both current and average pose axes overlay on the image."""
        out = img_bgr.copy()
        dark_red, dark_grn, dark_blu = (0, 0, 180), (0, 180, 0), (180, 0, 0)
        lite_red, lite_grn, lite_blu = (144, 144, 255), (144, 255, 144), (255, 144, 144)

        self._draw_pose_axes(out, T_avg, len_avg, dark_red, dark_grn, dark_blu, 3)
        self._draw_pose_axes(out, T_curr, len_cur, lite_red, lite_grn, lite_blu, 2)
        return out

    def _draw_pose_axes(self, img: np.ndarray, T: np.ndarray, axis_len: float,
                        color_x, color_y, color_z, thickness: int = 2):
        """Draw 3D pose axes projected onto the image plane."""
        Rm, t = T[:3, :3], T[:3, 3]
        O_f = np.zeros(3)
        X_f, Y_f, Z_f = np.array([axis_len, 0, 0]), np.array([0, axis_len, 0]), np.array([0, 0, axis_len])
        O_c, X_c, Y_c, Z_c = Rm @ O_f + t, Rm @ X_f + t, Rm @ Y_f + t, Rm @ Z_f + t
        okO, O_p = self._project(O_c)
        okX, X_p = self._project(X_c)
        okY, Y_p = self._project(Y_c)
        okZ, Z_p = self._project(Z_c)
        if not (okO and okX and okY and okZ):
            return
        O = tuple(map(int, O_p))
        cv2.arrowedLine(img, O, tuple(map(int, X_p)), color_x, thickness, tipLength=0.2)
        cv2.arrowedLine(img, O, tuple(map(int, Y_p)), color_y, thickness, tipLength=0.2)
        cv2.arrowedLine(img, O, tuple(map(int, Z_p)), color_z, thickness, tipLength=0.2)

    def _project(self, P_c: np.ndarray):
        """Project a 3D point in camera frame to 2D image coordinates."""
        Z = float(P_c[2])
        if Z <= 0.0 or not np.isfinite(Z):
            return False, (0, 0)
        u = self.fx * (float(P_c[0]) / Z) + self.cx
        v = self.fy * (float(P_c[1]) / Z) + self.cy
        if not (np.isfinite(u) and np.isfinite(v)):
            return False, (0, 0)
        return True, (u, v)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerlessPoseAverager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
