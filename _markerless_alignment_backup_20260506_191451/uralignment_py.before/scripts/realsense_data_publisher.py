#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from typing import Optional, Tuple

import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge


class RealSenseNode(Node):
    """
    Node that publishes:
      - Color image (sensor_msgs/Image)
      - Accelerometer unit vector (geometry_msgs/Vector3Stamped)
    Requires an Intel RealSense device with IMU support.
    """

    # ======================
    # Lifecycle
    # ======================
    def __init__(self) -> None:
        """Initialize node, parameters, publishers, and start pipeline."""
        super().__init__("realsense_data_publisher")

        self._declare_params()
        self._read_params()
        self._bridge = CvBridge()

        self._pipeline: Optional[rs.pipeline] = None
        self._has_accel: bool = False
        self._ema_vec: Optional[Tuple[float, float, float]] = None

        self._setup_publishers()
        self._start_pipeline()
        self._start_timer()

    def destroy_node(self):
        """Stop pipeline before destroying node."""
        self._stop_pipeline()
        return super().destroy_node()

    # ======================
    # Parameters
    # ======================
    def _declare_params(self) -> None:
        """Declare default ROS parameters."""
        self.declare_parameter("width", 1920)   # color frame width
        self.declare_parameter("height", 1080)  # color frame height
        self.declare_parameter("fps", 30)       # color FPS
        self.declare_parameter("imu_fps", 100)  # IMU FPS
        self.declare_parameter("frame_id", "camera_link")

        # ROS topics
        self.image_topic = "/realsense/image_raw"
        self.accel_topic = "/realsense/accel"

        # EMA smoothing factor (0..1)
        self.alpha = 0.2

    def _read_params(self) -> None:
        """Read parameters from ROS parameter server."""
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = int(self.get_parameter("fps").value)
        self.imu_fps = int(self.get_parameter("imu_fps").value)

    # ======================
    # ROS I/O
    # ======================
    def _setup_publishers(self) -> None:
        """Create publishers for image and accelerometer data."""
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub_image = self.create_publisher(Image, self.image_topic, image_qos)
        self.pub_accel = self.create_publisher(Vector3Stamped, self.accel_topic, sensor_qos)

    def _start_timer(self) -> None:
        """Start timer callback at the given FPS."""
        period = 1.0 / float(self.fps) if self.fps > 0 else 1.0 / 30.0
        self.create_timer(period, self._on_timer)

    # ======================
    # RealSense setup
    # ======================
    def _start_pipeline(self) -> None:
        """Start RealSense pipeline with color and IMU. Crash if IMU not available."""
        ctx = rs.context()
        devs = ctx.query_devices()
        if len(devs) == 0:
            raise RuntimeError("No RealSense device found.")

        dev = devs[0]
        serial = dev.get_info(rs.camera_info.serial_number)
        product_line = dev.get_info(rs.camera_info.product_line)
        sensors = dev.query_sensors()

        # Require IMU sensor
        has_motion = any(
            s.get_info(rs.camera_info.name).lower().startswith("motion")
            for s in sensors
        )
        if not has_motion:
            raise RuntimeError(f"Device {product_line} (S/N {serial}) has NO IMU.")
        
        self._has_accel = True 

        # Configure both color and accelerometer
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        cfg.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, self.imu_fps)

        self._pipeline = rs.pipeline()
        self._pipeline.start(cfg)

        self.get_logger().info(f"Started realsense pipeline \n---> Camera frame {self.width}x{self.height} | IMU @{self.fps} [fps]")

    def _stop_pipeline(self) -> None:
        """Stop RealSense pipeline safely."""
        try:
            if self._pipeline is not None:
                self._pipeline.stop()
        except Exception:
            pass

    # ======================
    # Timer callback
    # ======================
    def _on_timer(self) -> None:
        """Timer callback: grab frames and publish image + accel data."""
        frames = self._pipeline.wait_for_frames(timeout_ms=1000)
        if frames is None: 
            raise RuntimeError("No frames received from pipeline.")
        self._publish_color(frames)
        if self._has_accel: 
            self._publish_accel(frames)

    # ======================
    # Publishers
    # ======================
    def _publish_color(self, frames: rs.composite_frame) -> None:
        """Publish a color image frame."""
        color = frames.get_color_frame()
        if not color:
            return
        img = np.asanyarray(color.get_data())

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_link"

        msg = self._bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header = header
        self.pub_image.publish(msg)

    def _publish_accel(self, frames: rs.composite_frame) -> None:
        """Publish unit vector from accelerometer data."""
        latest_motion = None
        for f in frames:  # search latest motion frame
            if f and f.is_motion_frame():
                m = f.as_motion_frame()
                if m.get_profile().stream_type() == rs.stream.accel:
                    latest_motion = m

        if latest_motion is None:
            return

        data = latest_motion.get_motion_data()
        ax, ay, az = float(data.x), float(data.y), float(data.z)

        # Apply EMA smoothing
        sx, sy, sz = self._ema(ax, ay, az)

        # Normalize vector to unit length
        unit = self._normalize(sx, sy, sz)
        if unit is None:
            return

        ux, uy, uz = unit
        out = Vector3Stamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "camera_link"
        out.vector.x = ux
        out.vector.y = uy
        out.vector.z = uz
        self.pub_accel.publish(out)

    # ======================
    # Math helpers
    # ======================
    def _ema(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """Apply exponential moving average to accel data."""
        if self._ema_vec is None:
            self._ema_vec = (x, y, z)
        else:
            a = self.alpha
            ex, ey, ez = self._ema_vec
            self._ema_vec = (
                a * x + (1.0 - a) * ex,
                a * y + (1.0 - a) * ey,
                a * z + (1.0 - a) * ez,
            )
        return self._ema_vec

    @staticmethod
    def _normalize(x: float, y: float, z: float) -> Optional[Tuple[float, float, float]]:
        """Normalize a 3D vector to unit length."""
        n = math.sqrt(x * x + y * y + z * z)
        if n < 1e-9:
            return None
        return (x / n, y / n, z / n)


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
