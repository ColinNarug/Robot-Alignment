#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Vector3Stamped
from std_msgs.msg import Bool

from spatialmath import SE3
from scipy.spatial.transform import Rotation as R

from ur_alignment.flange_pose_estimator import FlangePoseEstimator


"""
This node performs markerless flange pose estimation.

It subscribes to camera images and accelerometer data, 
aligns the estimator with the IMU direction at each cycle, 
calculates pose with FlangePoseEstimator class, 
and publishes the estimated 6D pose together with diagnostic outputs.
"""


class MarkerlessPoseEstimator(Node):

    # ======================
    # Lifecycle
    # ======================
    def __init__(self) -> None:
        """Initialize node and setup config, state, QoS, and I/O."""
        super().__init__('markerless_pose_estimator')

        self._declare_params()
        cfg = self._load_config()
        self._init_core(cfg)
        self._init_state()
        self._init_qos()
        self._init_io()

        self.get_logger().info("Markerless pose estimator node ready. Waiting for imu data from realsense…")

    # ======================
    # Parameters & Config
    # ======================
    def _declare_params(self) -> None:
        """Declare ROS parameters for node behavior."""
        self.declare_parameter('publish_debug_image', True)

    def _load_config(self) -> dict:
        """Load YAML configuration file for flange pose estimator."""
        package_share = get_package_share_directory('ur_alignment')
        config_path = os.path.join(package_share, 'config', 'flange_pose_estimator_config.yaml')
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)

    # ======================
    # Core objects & state
    # ======================
    def _init_core(self, cfg: dict) -> None:
        """Initialize core objects like CvBridge and FlangePoseEstimator."""
        self.bridge = CvBridge()
        self.flange_pose_estimator = FlangePoseEstimator(cfg)

    def _init_state(self) -> None:
        """Initialize state variables for pose estimation pipeline."""
        self.initialized: bool = False
        self.success: bool = False         # Pose estimation success flag
        self.T_camera_flange: SE3 = None   # Pose estimation result
        
        self.publish_debug_image: bool = bool(self.get_parameter('publish_debug_image').get_parameter_value().bool_value)

        self._sub_image = None
        self._sub_accel_dir = None

        self.T_camera_flange_pub = None
        self.pose_success_pub = None
        self.debug_image_pub = None

        # Cache last valid accelerometer unit vector (used every frame)
        self._last_accel_unit: np.ndarray | None = None

    def _init_qos(self) -> None:
        """Setup QoS profiles for image and sensor topics."""
        self.qos_image = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )
        self.qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            durability=DurabilityPolicy.VOLATILE
        )

    # ======================
    # ROS I/O
    # ======================
    def _init_io(self) -> None:
        """Initialize subscriptions and publishers for the node."""
        # Acceleration direction (unit vector) from your RealSense node
        self._sub_accel_dir = self.create_subscription(
            Vector3Stamped, '/realsense/accel', self._accel_dir_cb, self.qos_sensor
        )

        # Image subscription
        self._sub_image = self.create_subscription(
            Image, '/realsense/image_raw', self._image_cb, self.qos_image
        )

        # Publishers
        self.T_camera_flange_pub = self.create_publisher(TransformStamped, '/cMo', 10)
        self.pose_success_pub = self.create_publisher(Bool, '/pose_estimation_success', 10)
        if self.publish_debug_image:
            self.debug_image_pub = self.create_publisher(Image, '/image_with_rf', 10)

    # ======================
    # Accel callback
    # ======================
    def _accel_dir_cb(self, msg: Vector3Stamped) -> None:
        """Update the last accel unit vector for per-frame alignment."""
        v = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=float)
        n = float(np.linalg.norm(v))
        if n <= 1e-9:
            return
        self._last_accel_unit = v / n
        self.initialized = True  # mark that we have a valid IMU reference

    # ======================
    # Image callback & pose
    # ======================
    def _image_cb(self, msg: Image) -> None:
        """Handle incoming images, align with IMU, estimate pose, and publish results."""
        self.start_t = time.perf_counter()

        # Require at least one IMU reading
        if not self.initialized or self._last_accel_unit is None:
            return

        # --- Always align with the latest IMU direction before estimating ---
        try:
            self.flange_pose_estimator.set_initial_pose(self._last_accel_unit)
        except Exception as e:
            self.get_logger().error(f"IMU alignment failed: {e}")
            self._report_success(False)
            return

        bgr = self._to_cv_bgr(msg)
        if bgr is None:
            self._report_success(False)
            return

        ok, T = self._estimate_pose(bgr)
        self.T_camera_flange = T if ok else None
        self._report_success(ok)

        if ok:
            self._publish_transform(msg.header)
            self._publish_debug_image_if_any(msg.header)
        else:
            self.get_logger().warn("Estimation failed")

    def _to_cv_bgr(self, msg: Image):
        """Convert ROS Image message to OpenCV BGR image."""
        try:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return None

    def _estimate_pose(self, bgr) -> tuple[bool, object]:
        """Run flange pose estimator on an OpenCV BGR frame."""
        try:
            ok, T = self.flange_pose_estimator.get_pose(bgr)
            return bool(ok), T
        except Exception: return False, None

    # ======================
    # Publishers
    # ======================
    def _publish_transform(self, header) -> None:
        """Publish pose transform as TransformStamped message."""
        if self.T_camera_flange is None:
            return

        t = self.T_camera_flange.t
        Rm = self.T_camera_flange.R
        rvec = R.from_matrix(Rm).as_rotvec()
        q = R.from_matrix(Rm).as_quat()

        tf_msg = TransformStamped()
        tf_msg.header = header
        tf_msg.header.frame_id = "camera_frame"
        tf_msg.child_frame_id = "flange_frame"

        tf_msg.transform.translation.x = float(t[0])
        tf_msg.transform.translation.y = float(t[1])
        tf_msg.transform.translation.z = float(t[2])

        tf_msg.transform.rotation.x = float(q[0])
        tf_msg.transform.rotation.y = float(q[1])
        tf_msg.transform.rotation.z = float(q[2])
        tf_msg.transform.rotation.w = float(q[3])

        self.T_camera_flange_pub.publish(tf_msg)

        # Pose log
        dt = time.perf_counter() - self.start_t
        hz = (1.0 / dt) if dt > 0 else 0.0
        log_str = (
            "\nESTIMATION SUCCESS:\n"
            f"t : [{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}] [m]\n"
            f"r : [{rvec[0]:.6f}, {rvec[1]:.6f}, {rvec[2]:.6f}] [rad]\n"
            f"rate : {hz:.2f} Hz\n"
            "#############################"
        )
        self.get_logger().info(log_str)

    def _publish_debug_image_if_any(self, header) -> None:
        """Publish debug image with reference frame if available."""
        if not self.publish_debug_image or self.debug_image_pub is None:
            return
        dbg = self.flange_pose_estimator.get_image_with_RF()
        if dbg is None:
            return
        img_msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
        img_msg.header = header
        self.debug_image_pub.publish(img_msg)

    # ======================
    # Helpers
    # ======================
    def _report_success(self, ok: bool) -> None:
        """Publish success flag of pose estimation."""
        self.success = bool(ok)
        msg = Bool()
        msg.data = self.success
        self.pose_success_pub.publish(msg)

    # ======================
    # Shutdown
    # ======================
    def destroy_node(self):
        """Clean up subscriptions before shutting down node."""
        try:
            if self._sub_accel_dir is not None:
                self.destroy_subscription(self._sub_accel_dir)
                self._sub_accel_dir = None
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MarkerlessPoseEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
