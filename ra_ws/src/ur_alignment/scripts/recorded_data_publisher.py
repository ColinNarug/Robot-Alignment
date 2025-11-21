#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import re
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2


"""
Node that:

- Publishes a sequence of PNG images on /realsense/image_raw.
- Publishes a fixed accelerometer vector on /realsense/accel.
- Parameter 'images_path': absolute/relative path; if not valid, falls back to <share>/ur_alignment/video_flange.
"""

class RecordedDataPublisher(Node):
    def __init__(self):
        super().__init__('recorded_data_publisher')

        # Parameters 
        self.declare_parameter('images_path', '')

        # Publishers
        self.pub_img = self.create_publisher(Image, '/realsense/image_raw', 10)
        self.pub_acc = self.create_publisher(Vector3Stamped, '/realsense/accel', 10)

        self.bridge = CvBridge()

        # Image folder
        images_path = self._resolve_images_path(
            self.get_parameter('images_path').get_parameter_value().string_value
        )

        # Collect frames
        self.image_files = self._collect_pngs(images_path)
        self._log_loaded(images_path, len(self.image_files))

        # Playback state
        self.idx = 0
        self.gravity_up = (0.0, 9.81, 0.0)  # m/s²

        # 30 Hz publishing timer
        self.timer = self.create_timer(1/10, self._on_timer)

    # ---------- path & loading ----------

    def _resolve_images_path(self, param_path: str) -> str:
        """Return a valid directory path for images, using package share as fallback."""
        p = os.path.expanduser(os.path.expandvars(param_path or ''))
        if p and (os.path.isabs(p) or os.path.isdir(p)):
            return p
        # fallback to installed share path
        share = get_package_share_directory('ur_alignment')
        return os.path.join(share, 'video_cavity')

    def _collect_pngs(self, folder: str):
        """Return a naturally-sorted list of .png file paths; [] if folder invalid."""
        if not os.path.isdir(folder):
            self.get_logger().error(f"Images path does not exist or is not a directory: {folder}")
            return []
        files = [os.path.join(folder, f) for f in os.listdir(folder) if f.lower().endswith('.png')]
        files.sort(key=self._frame_key)  # natural sort by first integer in name
        return files

    def _frame_key(self, path: str) -> int:
        m = re.search(r'(\d+)', os.path.basename(path))
        return int(m.group(1)) if m else 0

    def _log_loaded(self, folder: str, n: int) -> None:
        self.get_logger().info(f"Loaded {n} images from {folder}")

    # ---------- timer & publishing ----------
    def _on_timer(self):
        self._publish_next_image()
        self._publish_accel()

    def _publish_next_image(self):
        if self.idx >= len(self.image_files):
            self.get_logger().info("All images published. Stopping timer.")
            self.timer.cancel()
            return

        path = self.image_files[self.idx]
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn(f"Failed to read image: {path}")
            self.idx += 1
            return

        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'realsense_color_frame'
        self.pub_img.publish(msg)

        self.get_logger().info(f"Published image: {os.path.basename(path)}")
        self.idx += 1

    def _publish_accel(self):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'realsense_accel_frame'
        msg.vector.x, msg.vector.y, msg.vector.z = self.gravity_up
        self.pub_acc.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RecordedDataPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
