#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os, cv2, yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from spatialmath import SE3
from scipy.spatial.transform import Rotation as Rsc

from ur_alignment.user_interface.display import Display
from ur_alignment.user_interface.plotter import Plotter

"""
Dashboard:
    1) Camera image with desired/current pose 
    2) Two stacked  plots: position error, orientation error, ... (possibly more in future)
    3) (IN THE FUTURE) Keyboard commands
"""

class UserInterface(Node):

    def __init__(self):
        super().__init__('user_interface')

        # --- params ---
        self.declare_parameter('img_width', 1280)
        self.declare_parameter('img_height', 720)
        self.img_w = int(self.get_parameter('img_width').value)
        self.img_h = int(self.get_parameter('img_height').value)

        # --- subs (callbacks only update state) ---
        qos_img = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        self.bridge = CvBridge()
        self.create_subscription(Image, '/realsense/image_raw', self._on_image, qos_img)
        self.create_subscription(TransformStamped, '/cMo', self._on_pose, 10)

        # --- subscribe to errors data ---
        self.create_subscription(Float64MultiArray, '/errors_xyz', self._on_err_xyz, 10)
        self.create_subscription(Float64MultiArray, '/error',      self._on_err_rot, 10)

        # --- state ---
        self.image_arrived = False
        self.latest_img: np.ndarray | None = None
        self.c_T: SE3 | None = None
        self.have_cMo = False

        # --- config: intrinsics + desired pose (only for drawing) ---
        self.K = self._load_intrinsics()
        self.display = Display(self.K)  
        self.cd_T = self._load_cdMo()       
        self.has_cdMo = self.cd_T is not None

        # --- plots (your Plotter) ---
        self.history_len = 200  # keep last N samples
        self.plot_w = 520 # plots width [pix]
        self.plot_pos = Plotter(self.history_len)
        self.plot_pos.add_series("tx (m)", "m",  (40, 40, 220))
        self.plot_pos.add_series("ty (m)", "m",  (40, 180, 40))
        self.plot_pos.add_series("tz (m)", "m",  (220, 140, 40))

        self.plot_ori = Plotter(self.history_len)
        self.plot_ori.add_series("rx (rad)", "rad", (40, 40, 220))
        self.plot_ori.add_series("ry (rad)", "rad", (40, 180, 40))
        self.plot_ori.add_series("rz (rad)", "rad", (220, 140, 40))

        # --- UI ---
        self.win = "Dashboard"
        cv2.namedWindow(self.win, cv2.WINDOW_NORMAL)
        self._last_shape = None

        # --- timer ---
        self.timer = self.create_timer(0.01, self._tick)
        self.get_logger().info("UI ready (reading errors from topics)")

    # ============================
    # Callbacks 
    # ============================

    def _on_image(self, msg: Image):
        """Store last image; set flag at first arrival."""
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if img.ndim == 2:
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            elif img.shape[2] == 4:
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            self.latest_img = img
            self.image_arrived = True
        except Exception:
            return  # keep previous frame/placeholder

    def _on_pose(self, msg: TransformStamped):
        """Store current pose (for drawing only)."""
        try:
            q = msg.transform.rotation
            Rm = Rsc.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            t = msg.transform.translation
            self.c_T = SE3.Rt(Rm, [t.x, t.y, t.z])
            self.have_cMo = True
        except Exception:
            return

        # NOTE: errors are NOT computed here anymore; they are read from topics.

    def _on_err_xyz(self, msg: Float64MultiArray):
        """Append position error [tx,ty,tz] to position plot."""
        try:
            data = list(msg.data)
            if len(data) >= 3:
                self.plot_pos.append_sample({
                    "tx (m)": float(data[0]),
                    "ty (m)": float(data[1]),
                    "tz (m)": float(data[2]),
                })
        except Exception:
            return

    def _on_err_rot(self, msg: Float64MultiArray):
        """Append orientation error [rx,ry,rz]. Uses indices 3:6 if len>=6, else first 3."""
        try:
            data = list(msg.data)
            if len(data) >= 6:
                rx, ry, rz = float(data[3]), float(data[4]), float(data[5])
            elif len(data) >= 3:
                rx, ry, rz = float(data[0]), float(data[1]), float(data[2])
            else:
                return
            self.plot_ori.append_sample({
                "rx (rad)": rx,
                "ry (rad)": ry,
                "rz (rad)": rz,
            })
        except Exception:
            return

    # ============================
    # Full render
    # ============================
    def _tick(self):
        """Render whole dashboard."""
        # base image
        if self.image_arrived and self.latest_img is not None:
            img = self.latest_img.copy()
            # draw poses BEFORE resizing (intrinsics match original size)
            self.display.set_frame(img)
            if self.has_cdMo:
                self.display.draw_pose(self.cd_T, alpha=0.4)   # desired (faint)
            if self.have_cMo and self.c_T is not None:
                self.display.draw_pose(self.c_T, alpha=1.0)   # current
            img_drawn = self.display.get_frame(copy=False)
        else:
            img_drawn = self._placeholder()

        # resize for UI layout
        left = cv2.resize(img_drawn, (self.img_w, self.img_h), cv2.INTER_AREA)

        # right column (two stacked plots)
        sep_h = 8
        top_h = (left.shape[0] - sep_h) // 2
        bot_h = left.shape[0] - sep_h - top_h

        top = np.zeros((top_h, self.plot_w, 3), np.uint8)
        bot = np.zeros((bot_h, self.plot_w, 3), np.uint8)
        self.plot_pos.set_canvas(top)
        self.plot_pos.render(title="Position error")
        self.plot_ori.set_canvas(bot)
        self.plot_ori.render(title="Orientation error")
        right = np.vstack([top, np.ones((sep_h, self.plot_w, 3), np.uint8) * 255, bot])

        # compose H
        if right.shape[0] != left.shape[0]:
            right = cv2.resize(right, (right.shape[1], left.shape[0]), cv2.INTER_AREA)
        sep = np.ones((left.shape[0], 10, 3), np.uint8) * 255
        dash = np.hstack([left, sep, right])

        # resize window once per shape change
        if self._last_shape != dash.shape:
            cv2.resizeWindow(self.win, dash.shape[1], dash.shape[0])
            self._last_shape = dash.shape

        cv2.imshow(self.win, dash)
        cv2.waitKey(1)


    # ============================
    # Helpers
    # ============================

    def _placeholder(self) -> np.ndarray:
        """Black image with centered text."""
        img = np.zeros((self.img_h, self.img_w, 3), np.uint8)
        text = "waiting for image!"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
        x = (self.img_w - tw) // 2
        y = (self.img_h + th) // 2
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
        return img

    def _load_intrinsics(self) -> np.ndarray:
        """Load the camera matrix flange_pose_estimator_config.yaml."""
        try:
            pkg = get_package_share_directory('ur_alignment')
            yaml_path = os.path.join(pkg, 'config', 'flange_pose_estimator_config.yaml')
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            cam = data["CAMERA_PARAMETERS"]
            fx, fy, ux, uy = float(cam["FX"]), float(cam["FY"]), float(cam["UX"]), float(cam["UY"])
            return np.array([[fx, 0, ux], [0, fy, uy], [0, 0, 1]], dtype=float)
        except Exception:
            self.get_logger().warn("Intrinsics load failed; using fallback K.")
            return np.array([[1000, 0, 640], [0, 1000, 360], [0, 0, 1]], dtype=float)

    def _find_share(self, *rel) -> str:
        """Return path inside package share"""
        try:
            share = get_package_share_directory('ur_alignment')
            return os.path.join(share, *rel)
        except Exception:
            return os.path.join(os.getcwd(), *rel)

    def _load_cdMo(self) -> SE3 | None:
        """Load desired pose from ur_cdMo.yaml"""
        path = self._find_share('config', 'ur_cdMo.yaml')
        if not os.path.isfile(path):
            return None
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            raw = data.get('data', [])
            if len(raw) < 6:
                return None
            vals = [float(item[0] if isinstance(item, list) else item) for item in raw[:6]]
            tvec = np.array(vals[:3], float)
            rvec = np.array(vals[3:], float)
            Rm = Rsc.from_rotvec(rvec).as_matrix()
            return SE3.Rt(Rm, tvec)
        except Exception:
            return None

# ============================
# main
# ============================

def main(args=None):
    rclpy.init(args=args)
    node = UserInterface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
