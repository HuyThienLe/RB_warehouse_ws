#!/usr/bin/env python3
"""web_bridge.py – ROS 2 Node làm cầu nối giữa Web Backend và Robot Middleware (Bài 8.1 & 8b).

Các nhiệm vụ chính:
- Lắng nghe /robot_status để cập nhật trạng thái robot thời gian thực (5-10 Hz).
- Lắng nghe /map (QoS TRANSIENT_LOCAL) chuyển đổi OccupancyGrid thành PNG trực tiếp trong bộ nhớ (Bài 8b.2).
- Biến đổi điểm LiDAR (/scan) sang hệ tọa độ map qua TF buffer với tần số 5 Hz (Bài 8b.4).
- Ghi nhận quỹ đạo di chuyển (trail) và đường đi Nav2 (/plan) (Bài 8b.3).
- Điều khiển Teleop (/cmd_vel_teleop) kèm cơ chế Watchdog an toàn (ngắt sau 0.3s không có lệnh) (Bài 8b.5).
- Tích hợp các service lưu bản đồ SLAM (SaveMap, SerializePoseGraph, Clear) (Bài 8b.5).
- Đẩy dữ liệu ra ngoài cho WebSocket thông qua callback thread-safe.
"""

import math
import os
import time
import threading
from typing import Optional, Callable, Dict, Any, Tuple, List

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan, Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import tf2_ros

from wh_interfaces.msg import RobotStatus, TagArray, TagDetection
from wh_interfaces.srv import SubmitTask, CancelTask
from slam_toolbox.srv import SaveMap, SerializePoseGraph, Clear
from wh_task_manager.inventory_db import InventoryDB


def make_placeholder_frame(text: str) -> bytes:
    """Sinh ảnh frame placeholder khi chưa nhận luồng camera."""
    img = np.zeros((240, 320, 3), dtype=np.uint8)
    img[:] = (20, 24, 32)
    cv2.rectangle(img, (2, 2), (317, 237), (0, 210, 255), 1)
    cv2.putText(img, text, (20, 115), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 210, 255), 1, cv2.LINE_AA)
    cv2.putText(img, "Waiting for video stream...", (20, 145), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (140, 160, 180), 1, cv2.LINE_AA)
    ok, buf = cv2.imencode(".jpg", img)
    return buf.tobytes() if ok else b""


def grid_to_png(msg: OccupancyGrid) -> bytes:
    """Chuyển đổi OccupancyGrid sang ảnh xám PNG chuẩn (Bài 8b.2)."""
    h = msg.info.height
    w = msg.info.width
    if h == 0 or w == 0 or len(msg.data) != h * w:
        return b""

    a = np.array(msg.data, dtype=np.int8).reshape(h, w)
    img = np.full(a.shape, 128, dtype=np.uint8)  # chưa biết (-1) = xám (128)
    img[a == 0] = 255                            # ô trống (0) = trắng (255)
    img[a >= 65] = 0                             # vật cản (>=65) = đen (0)
    img = np.flipud(img)                         # ROS: gốc ở dưới-trái; Ảnh: gốc ở trên-trái

    ok, buf = cv2.imencode(".png", img)
    return buf.tobytes() if ok else b""


class WebBridge(Node):
    """Cầu nối giữa ROS 2 và FastAPI."""

    def __init__(
        self,
        db: InventoryDB,
        push_callback: Optional[Callable[[Dict[str, Any]], None]] = None,
    ):
        super().__init__(
            "web_bridge_node",
            parameter_overrides=[
                rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
            ]
        )
        self.db = db
        self.push_callback = push_callback
        self.cbg = ReentrantCallbackGroup()

        # Trạng thái robot
        self.latest_status: Dict[str, Any] = {
            "x": -2.40,
            "y": 0.00,
            "yaw": 0.00,
            "state": "IDLE",
            "task_id": 0,
            "slot_id": "",
            "battery_pct": 100.0,
            "holding_box": False,
            "held_box_tag": 0,
            "last_error": "",
            "linear_speed": 0.0,
            "angular_speed": 0.0,
            "is_estopped": False,
        }
        self.prev_state = "IDLE"
        self.prev_last_error = ""
        self.last_push_time = 0.0
        self.is_estopped = False
        self.lock = threading.Lock()

        # Chế độ vận hành: "navigation" | "mapping"
        self.current_mode = "navigation"

        # Bản đồ sống (Live Map)
        self.map_version = 0
        self.live_map_png: Optional[bytes] = None
        self.live_map_meta: Dict[str, Any] = {
            "version": 0,
            "width": 0,
            "height": 0,
            "resolution": 0.025,
            "origin_x": 0.0,
            "origin_y": 0.0,
        }
        self.last_map_time = 0.0

        # Lớp LiDAR scan & Quỹ đạo & Đường đi
        self.last_scan_time = 0.0
        self.trail: List[List[float]] = []
        self.last_trail_x = -999.0
        self.last_trail_y = -999.0
        self.path_points: List[List[float]] = []

        # Teleop Watchdog
        self.last_teleop_time = 0.0
        self.is_teleop_active = False

        # TF2 Buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS Latched (TRANSIENT_LOCAL + RELIABLE, depth 1)
        qos_latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ── 1. Subscribers ───────────────────────────────────────────────────
        self.sub_status = self.create_subscription(
            RobotStatus,
            "/robot_status",
            self.cb_robot_status,
            10,
            callback_group=self.cbg,
        )
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cb_cmd_vel,
            10,
            callback_group=self.cbg,
        )
        self.sub_mode = self.create_subscription(
            String,
            "/wh/mode",
            self.cb_mode,
            qos_latched,
            callback_group=self.cbg,
        )
        self.sub_map = self.create_subscription(
            OccupancyGrid,
            "/map",
            self.cb_map,
            qos_latched,
            callback_group=self.cbg,
        )
        self.sub_scan = self.create_subscription(
            LaserScan,
            "/scan",
            self.cb_scan,
            10,
            callback_group=self.cbg,
        )
        self.sub_plan = self.create_subscription(
            Path,
            "/plan",
            self.cb_plan,
            10,
            callback_group=self.cbg,
        )

        # ── 1b. Camera & Vision Subscriptions ────────────────────────────────
        self.bridge = CvBridge()
        self.latest_dock_jpeg: Optional[bytes] = None
        self.latest_hand_jpeg: Optional[bytes] = None
        self.last_dock_img_time = 0.0
        self.last_hand_img_time = 0.0
        self.dock_tags_detected: List[Dict[str, Any]] = []
        self.hand_tags_detected: List[Dict[str, Any]] = []

        self.sub_dock_debug = self.create_subscription(
            Image, "/vision/debug_dock", self.cb_dock_debug_image, 5, callback_group=self.cbg
        )
        self.sub_dock_raw = self.create_subscription(
            Image, "/dock_camera/image_raw", self.cb_dock_raw_image, 5, callback_group=self.cbg
        )
        self.sub_hand_debug = self.create_subscription(
            Image, "/vision/debug_hand", self.cb_hand_debug_image, 5, callback_group=self.cbg
        )
        self.sub_hand_raw = self.create_subscription(
            Image, "/hand_camera/image_raw", self.cb_hand_raw_image, 5, callback_group=self.cbg
        )
        self.sub_vision_tags = self.create_subscription(
            TagArray, "/vision/tags", self.cb_vision_tags, 10, callback_group=self.cbg
        )

        # ── 2. Publishers ────────────────────────────────────────────────────
        self.pub_estop = self.create_publisher(Bool, "/estop", 10)
        self.pub_estop_reset = self.create_publisher(Empty, "/estop_reset", 10)
        self.pub_teleop = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.pub_arm_traj = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)

        # ── 3. Timers ────────────────────────────────────────────────────────
        self.timer_watchdog = self.create_timer(0.1, self.teleop_watchdog)
        self.has_tf_pose = False
        self.timer_tf_pose = self.create_timer(0.05, self.update_tf_pose, callback_group=self.cbg)

        # ── 4. Service Clients ───────────────────────────────────────────────
        self.cli_submit = self.create_client(
            SubmitTask,
            "/wh/submit_task",
            callback_group=self.cbg,
        )
        self.cli_cancel = self.create_client(
            CancelTask,
            "/wh/cancel_task",
            callback_group=self.cbg,
        )
        self.cli_save_map = self.create_client(
            SaveMap,
            "/slam_toolbox/save_map",
            callback_group=self.cbg,
        )
        self.cli_serialize_map = self.create_client(
            SerializePoseGraph,
            "/slam_toolbox/serialize_map",
            callback_group=self.cbg,
        )
        self.cli_clear_slam = self.create_client(
            Clear,
            "/slam_toolbox/clear",
            callback_group=self.cbg,
        )
        self.cli_gripper = self.create_client(
            SetBool,
            "/gripper/switch",
            callback_group=self.cbg,
        )
        self.cli_clear_out = self.create_client(
            Trigger,
            "/wh/clear_station_out",
            callback_group=self.cbg,
        )
        self.cli_replenish_in = self.create_client(
            Trigger,
            "/wh/replenish_station_in",
            callback_group=self.cbg,
        )

        self.get_logger().info("WebBridge node (Vision & Arm Enhanced) khởi động thành công.")

    def push(self, msg_type: str, data: Any):
        """Đẩy thông điệp WebSocket dạng {'type': ..., 'data': ...}."""
        if self.push_callback:
            try:
                self.push_callback({"type": msg_type, "data": data})
            except Exception as e:
                self.get_logger().error(f"Lỗi khi push thông điệp WebSocket: {e}")

    # ── Callbacks ────────────────────────────────────────────────────────────
    def cb_cmd_vel(self, msg: Twist):
        with self.lock:
            self.latest_status["linear_speed"] = float(msg.linear.x)
            self.latest_status["angular_speed"] = float(msg.angular.z)

    def cb_mode(self, msg: String):
        val = msg.data.strip().lower()
        if val in ("navigation", "mapping") and val != self.current_mode:
            self.current_mode = val
            self.get_logger().info(f"Chế độ vận hành cập nhật: '{self.current_mode.upper()}'")
            self.push("mode", {"mode": self.current_mode})

    def cb_map(self, msg: OccupancyGrid):
        now = time.time()
        # Giới hạn tần suất tối đa 1 lần/giây (Bài 8b.2)
        if now - self.last_map_time < 1.0 and self.map_version > 0:
            return
        self.last_map_time = now

        try:
            png_bytes = grid_to_png(msg)
            if not png_bytes:
                return

            with self.lock:
                self.map_version += 1
                self.live_map_png = png_bytes
                self.live_map_meta = {
                    "version": self.map_version,
                    "width": int(msg.info.width),
                    "height": int(msg.info.height),
                    "resolution": float(msg.info.resolution),
                    "origin_x": float(msg.info.origin.position.x),
                    "origin_y": float(msg.info.origin.position.y),
                }
                meta_copy = dict(self.live_map_meta)

            self.push("map_update", meta_copy)
        except Exception as e:
            self.get_logger().warn(f"Lỗi chuyển đổi bản đồ: {e}")

    def cb_scan(self, msg: LaserScan):
        now = time.time()
        # Giới hạn tần số gửi 5 Hz (cứ >= 0.2s một lần)
        if now - self.last_scan_time < 0.2:
            return
        self.last_scan_time = now

        try:
            frame_id = msg.header.frame_id or "lidar_link"
            t = self.tf_buffer.lookup_transform("map", frame_id, rclpy.time.Time())
            tx = t.transform.translation.x
            ty = t.transform.translation.y
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            pts = []
            step = 2  # Lấy mẫu thưa
            for i in range(0, len(msg.ranges), step):
                r = msg.ranges[i]
                if not (msg.range_min < r < msg.range_max):
                    continue
                a = msg.angle_min + i * msg.angle_increment + yaw
                pts.append([round(tx + r * math.cos(a), 3), round(ty + r * math.sin(a), 3)])
                if len(pts) >= 180:
                    break

            if pts:
                self.push("scan", {"points": pts})
        except Exception:
            # Bỏ qua khi TF map->lidar_link chưa sẵn sàng, không spam warning
            pass

    def cb_plan(self, msg: Path):
        pts = []
        n = len(msg.poses)
        if n == 0:
            self.push("path", {"points": []})
            return

        step = max(1, n // 100)
        for i in range(0, n, step):
            p = msg.poses[i].pose.position
            pts.append([round(p.x, 3), round(p.y, 3)])
        p_last = msg.poses[-1].pose.position
        pts.append([round(p_last.x, 3), round(p_last.y, 3)])
        self.push("path", {"points": pts})

    def update_tf_pose(self):
        """Lấy pose map -> base_footprint từ TF2 tại thời điểm mới nhất (chuẩn hóa theo Bài 10.5.1)."""
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            x = float(t.transform.translation.x)
            y = float(t.transform.translation.y)
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            with self.lock:
                self.latest_status["x"] = x
                self.latest_status["y"] = y
                self.latest_status["yaw"] = yaw
                self.has_tf_pose = True

            dist_moved = math.hypot(x - self.last_trail_x, y - self.last_trail_y)
            if dist_moved >= 0.05:
                self.last_trail_x = x
                self.last_trail_y = y
                self.trail.append([round(x, 3), round(y, 3)])
                if len(self.trail) > 1000:
                    self.trail.pop(0)
                self.push("trail", {"points": self.trail})
        except Exception:
            pass

    def cb_robot_status(self, msg: RobotStatus):
        now = time.time()
        curr_state = msg.state
        curr_task_id = msg.task_id
        curr_slot_id = msg.slot_id or ""
        curr_error = msg.last_error or ""

        with self.lock:
            effective_state = "ESTOP" if self.is_estopped else curr_state
            status_update = {
                "state": effective_state,
                "task_id": int(curr_task_id),
                "slot_id": curr_slot_id,
                "battery_pct": float(msg.battery_pct),
                "holding_box": bool(msg.holding_box),
                "held_box_tag": int(msg.held_box_tag),
                "last_error": curr_error,
                "is_estopped": self.is_estopped,
            }
            # Chỉ ghi đè x, y, yaw từ topic nếu TF map->base_footprint chưa sẵn sàng
            if not self.has_tf_pose:
                status_update.update({
                    "x": float(msg.x),
                    "y": float(msg.y),
                    "yaw": float(msg.yaw),
                })
            self.latest_status.update(status_update)
            if curr_state == "ESTOP":
                self.is_estopped = True

        # Nếu chưa có TF, cập nhật trail từ topic
        if not self.has_tf_pose:
            rx = float(msg.x)
            ry = float(msg.y)
            dist_moved = math.hypot(rx - self.last_trail_x, ry - self.last_trail_y)
            if dist_moved >= 0.05:
                self.last_trail_x = rx
                self.last_trail_y = ry
                self.trail.append([round(rx, 3), round(ry, 3)])
                if len(self.trail) > 1000:
                    self.trail.pop(0)
                self.push("trail", {"points": self.trail})

        # 1. Phát hiện sự kiện chuyển trạng thái nhiệm vụ (FSM Step)
        if curr_state != self.prev_state:
            self.get_logger().info(f"FSM State thay đổi: {self.prev_state} -> {curr_state}")
            self.push("task", {
                "task_id": curr_task_id,
                "state": curr_state,
                "slot_id": curr_slot_id,
                "last_error": curr_error,
            })

            # Khi kết thúc nhiệm vụ (chuyển sang DONE hoặc IDLE)
            if curr_state in ("DONE", "IDLE") and self.prev_state not in ("DONE", "IDLE"):
                self.push("inventory", self.db.get_inventory())
                task_info = self.db.get_task(curr_task_id) if curr_task_id > 0 else None
                task_status = task_info.get("status") if task_info else ("FAILED" if curr_error else "SUCCEEDED")
                task_err = task_info.get("error") if task_info else curr_error

                if task_status == "SUCCEEDED" and not task_err:
                    self.push("notification", {
                        "level": "info",
                        "text": f"Nhiệm vụ #{curr_task_id} đã hoàn tất thành công!",
                    })
                elif task_status == "CANCELLED":
                    self.push("notification", {
                        "level": "warn",
                        "text": f"Nhiệm vụ #{curr_task_id} đã bị hủy bỏ.",
                    })
                else:
                    self.push("notification", {
                        "level": "error",
                        "text": f"Nhiệm vụ #{curr_task_id} THẤT BẠI: {task_err or 'Lỗi thực thi'}",
                    })
                self._check_low_stock()

            elif curr_state == "ESTOP":
                self.push("notification", {
                    "level": "error",
                    "text": "CẢNH BÁO: Robot đang ở trạng thái DỪNG KHẨN CẤP (E-STOP)!",
                })

            self.prev_state = curr_state

        # 2. Phát hiện lỗi mới phát sinh
        if curr_error and curr_error != self.prev_last_error:
            self.push("notification", {
                "level": "warn",
                "text": f"Nhiệm vụ #{curr_task_id} gặp lỗi: {curr_error}",
            })
            self.prev_last_error = curr_error

        # 3. Tần số phát dữ liệu robot giới hạn ở ~10 Hz (cứ >= 0.1s gửi 1 lần)
        if now - self.last_push_time >= 0.1:
            self.last_push_time = now
            self.push("robot", self.get_robot_status())

    def _check_low_stock(self):
        """Kiểm tra và bắn thông báo nếu mặt hàng chạm ngưỡng tồn thấp."""
        try:
            inv = self.db.get_inventory()
            for item in inv.get("items", []):
                if item.get("is_low_stock"):
                    self.push("notification", {
                        "level": "warn",
                        "text": f"Sắp hết hàng: '{item['name']}' ({item['item_code']}) hiện chỉ còn {item['in_stock']} hộp (ngưỡng: {item['low_stock_threshold']})!",
                    })
        except Exception as e:
            self.get_logger().warn(f"Lỗi kiểm tra low stock: {e}")

    # ── Điều khiển E-Stop ────────────────────────────────────────────────────
    def trigger_estop(self):
        """Kích hoạt E-Stop dừng khẩn cấp."""
        msg = Bool(data=True)
        self.pub_estop.publish(msg)
        with self.lock:
            self.is_estopped = True
            self.latest_status["state"] = "ESTOP"
            self.latest_status["linear_speed"] = 0.0
            self.latest_status["angular_speed"] = 0.0
        self.push("notification", {
            "level": "error",
            "text": "Lệnh DỪNG KHẨN CẤP (E-STOP) đã được kích hoạt từ Web!",
        })
        self.push("robot", self.get_robot_status())

    def reset_estop(self):
        """Giải phóng E-Stop sau khi xác nhận an toàn."""
        msg = Empty()
        self.pub_estop_reset.publish(msg)
        with self.lock:
            self.is_estopped = False
            self.latest_status["state"] = "IDLE"
        self.push("notification", {
            "level": "info",
            "text": "Đã khôi phục trạng thái E-Stop. Robot sẵn sàng nhận lệnh mới.",
        })
        self.push("robot", self.get_robot_status())

    # ── Teleop & Watchdog An Toàn (Bài 8b.5) ──────────────────────────────────
    def send_teleop(self, vx: float, vy: float, wz: float) -> bool:
        """Phát lệnh vận tốc thủ công, giới hạn an toàn <= 0.25 m/s, <= 0.6 rad/s."""
        cmd = Twist()
        cmd.linear.x = max(-0.25, min(0.25, float(vx)))
        cmd.linear.y = max(-0.25, min(0.25, float(vy)))
        cmd.angular.z = max(-0.60, min(0.60, float(wz)))
        self.pub_teleop.publish(cmd)

        with self.lock:
            self.last_teleop_time = time.time()
            self.is_teleop_active = (abs(cmd.linear.x) > 0.001 or abs(cmd.linear.y) > 0.001 or abs(cmd.angular.z) > 0.001)
        return True

    def teleop_watchdog(self):
        """Watchdog: mất lệnh quá 0.3s thì tự động publish vận tốc 0 dừng robot."""
        with self.lock:
            if self.is_teleop_active and (time.time() - self.last_teleop_time > 0.3):
                stop_cmd = Twist()
                self.pub_teleop.publish(stop_cmd)
                self.is_teleop_active = False

    # ── Điều Khiển Dựng Bản Đồ SLAM (Bài 8b.5) ───────────────────────────────
    def save_slam_map(self, map_name: str) -> Tuple[bool, str, str]:
        """Gọi service slam_toolbox/save_map hoặc xuất ảnh PGM/YAML."""
        if not map_name:
            map_name = "warehouse_map"

        maps_dir = "/home/lehuythien/warehouse_ws/src/wh_navigation/maps"
        os.makedirs(maps_dir, exist_ok=True)
        target_path_base = os.path.join(maps_dir, map_name)

        # 1. Gọi service của slam_toolbox nếu có
        if self.cli_save_map.wait_for_service(timeout_sec=1.0):
            req = SaveMap.Request()
            req.name.data = target_path_base
            future = self.cli_save_map.call_async(req)
            t0 = time.time()
            while not future.done() and (time.time() - t0 < 4.0):
                time.sleep(0.05)
            if future.done() and future.result():
                res = future.result()
                if res.result == 0:
                    return True, f"Đã lưu bản đồ thành công: {target_path_base}.pgm", target_path_base
                return False, f"Lỗi lưu map từ slam_toolbox: mã {res.result}", ""

        # 2. Fallback ghi trực tiếp từ ảnh live_map_png hiện hành hoặc cached_map_png
        target_png = self.live_map_png
        if not target_png:
            import wh_web.app as app_mod
            target_png = getattr(app_mod, "cached_map_png", None)

        if target_png:
            pgm_path = target_path_base + ".pgm"
            yaml_path = target_path_base + ".yaml"
            nparr = np.frombuffer(target_png, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE)
            img_ros = np.flipud(img)  # Chuyển về hệ tọa độ ROS
            cv2.imwrite(pgm_path, img_ros)

            meta = self.live_map_meta
            res_val = meta.get("resolution", 0.025)
            ox_val = meta.get("origin_x", -3.5)
            oy_val = meta.get("origin_y", -2.5)
            with open(yaml_path, "w", encoding="utf-8") as f:
                f.write(f"image: {map_name}.pgm\n")
                f.write(f"resolution: {res_val}\n")
                f.write(f"origin: [{ox_val}, {oy_val}, 0.0]\n")
                f.write("negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
            return True, f"Đã lưu bản đồ trực tiếp: {pgm_path}", target_path_base

        return False, "Chưa nhận được dữ liệu bản đồ để lưu", ""

    def serialize_slam_map(self, filename: str) -> Tuple[bool, str]:
        """Lưu pose graph SLAM để mở lại lần sau."""
        if not filename:
            filename = "slam_pose_graph"
        maps_dir = "/home/lehuythien/warehouse_ws/src/wh_navigation/maps"
        os.makedirs(maps_dir, exist_ok=True)
        full_path = os.path.join(maps_dir, filename)

        if self.cli_serialize_map.wait_for_service(timeout_sec=1.0):
            req = SerializePoseGraph.Request()
            req.filename = full_path
            future = self.cli_serialize_map.call_async(req)
            t0 = time.time()
            while not future.done() and (time.time() - t0 < 4.0):
                time.sleep(0.05)
            if future.done() and future.result():
                res = future.result()
                if res.result == 0:
                    return True, f"Đã lưu pose graph vào {full_path}"
                return False, f"Lỗi serialize pose graph: mã {res.result}"

        return False, "Service serialize_map của slam_toolbox không khả dụng"

    def clear_slam(self) -> Tuple[bool, str]:
        """Xóa phiên SLAM hiện tại."""
        if self.cli_clear_slam.wait_for_service(timeout_sec=1.0):
            req = Clear.Request()
            future = self.cli_clear_slam.call_async(req)
            t0 = time.time()
            while not future.done() and (time.time() - t0 < 3.0):
                time.sleep(0.05)
            if future.done():
                with self.lock:
                    self.trail.clear()
                return True, "Đã đặt lại phiên SLAM thành công"
        return False, "Service clear SLAM không khả dụng"

    # ── Giao tiếp Task Manager ───────────────────────────────────────────────
    def submit_task(
        self,
        task_type: str,
        item_code: str,
        quantity: int = 1,
        priority: int = 1,
        slot_id: Optional[str] = None,
    ) -> Tuple[bool, int, str]:
        """Gửi lệnh nhập/xuất tới Task Manager qua ROS service (hoặc CSDL trực tiếp nếu có ô chỉ định)."""
        # Nếu có ô chỉ định cụ thể, ghi trực tiếp vào CSDL để truyền kèm slot_id
        if slot_id or not self.cli_submit.wait_for_service(timeout_sec=0.5):
            first_id = 0
            for i in range(quantity):
                tid = self.db.create_task(task_type.upper(), item_code, priority=priority, slot_id=slot_id)
                if i == 0:
                    first_id = tid
            slot_str = f" vào ô {slot_id}" if slot_id else ""
            return True, first_id, f"Tiếp nhận {quantity} đơn {task_type} cho '{item_code}'{slot_str} (Đơn #{first_id})"

        req = SubmitTask.Request()
        req.task_type = task_type.upper()
        req.item_code = item_code
        req.quantity = int(quantity)
        req.priority = int(priority)

        future = self.cli_submit.call_async(req)
        t0 = time.time()
        while not future.done() and (time.time() - t0 < 3.0):
            time.sleep(0.02)

        if future.done() and future.result():
            res = future.result()
            return res.success, res.task_id, res.message
        return False, 0, "Quá thời gian chờ phản hồi từ Task Manager"

    def cancel_task(self, task_id: int) -> Tuple[bool, str]:
        if self.cli_cancel.wait_for_service(timeout_sec=0.5):
            req = CancelTask.Request()
            req.task_id = int(task_id)

            future = self.cli_cancel.call_async(req)
            t0 = time.time()
            while not future.done() and (time.time() - t0 < 3.0):
                time.sleep(0.02)

            if future.done() and future.result():
                res = future.result()
                return res.success, res.message
            return False, "Quá thời gian chờ hủy nhiệm vụ từ Task Manager"

        ok = self.db.cancel_task(task_id)
        return ok, f"Đã hủy nhiệm vụ #{task_id}" if ok else f"Không thể hủy #{task_id}"

    def get_robot_status(self) -> Dict[str, Any]:
        with self.lock:
            st = dict(self.latest_status)
            if self.is_estopped:
                st["state"] = "ESTOP"
            st["is_estopped"] = self.is_estopped
            return st

    def get_live_map_meta(self) -> Dict[str, Any]:
        with self.lock:
            return dict(self.live_map_meta)

    def get_live_map_png(self) -> Optional[bytes]:
        with self.lock:
            return self.live_map_png

    # ── Camera & Vision Handlers ─────────────────────────────────────────────
    def cb_vision_tags(self, msg: TagArray):
        cam = msg.camera
        tag_list = []
        for t in msg.tags:
            tag_list.append({
                "id": int(t.id),
                "type": "shelf" if t.id < 20 else ("station" if t.id in (30, 31) else "box"),
                "reproj_error": round(float(getattr(t, "reproj_error_px", getattr(t, "reproj_error", 0.0))), 2),
                "dist": round(math.sqrt(t.pose_base.pose.position.x**2 + t.pose_base.pose.position.y**2), 3),
            })
        with self.lock:
            if "dock" in cam:
                self.dock_tags_detected = tag_list
            elif "hand" in cam:
                self.hand_tags_detected = tag_list

    def cb_dock_debug_image(self, msg: Image):
        self._process_image(msg, is_dock=True)

    def cb_dock_raw_image(self, msg: Image):
        if time.time() - self.last_dock_img_time > 0.5:
            self._process_image(msg, is_dock=True)

    def cb_hand_debug_image(self, msg: Image):
        self._process_image(msg, is_dock=False)

    def cb_hand_raw_image(self, msg: Image):
        if time.time() - self.last_hand_img_time > 0.5:
            self._process_image(msg, is_dock=False)

    def _process_image(self, msg: Image, is_dock: bool):
        now = time.time()
        last_t = self.last_dock_img_time if is_dock else self.last_hand_img_time
        if now - last_t < 0.06:
            return
        if is_dock:
            self.last_dock_img_time = now
        else:
            self.last_hand_img_time = now

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            ok, buf = cv2.imencode(".jpg", cv_img, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
            if ok:
                with self.lock:
                    if is_dock:
                        self.latest_dock_jpeg = buf.tobytes()
                    else:
                        self.latest_hand_jpeg = buf.tobytes()
        except Exception:
            pass

    def get_dock_frame(self) -> bytes:
        with self.lock:
            if self.latest_dock_jpeg:
                return self.latest_dock_jpeg
        return make_placeholder_frame("DOCK CAMERA: Đang kết nối...")

    def get_hand_frame(self) -> bytes:
        with self.lock:
            if self.latest_hand_jpeg:
                return self.latest_hand_jpeg
        return make_placeholder_frame("HAND CAMERA: Đang kết nối...")

    def get_vision_telemetry(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "dock_tags": list(self.dock_tags_detected),
                "hand_tags": list(self.hand_tags_detected),
                "has_dock_feed": bool(self.latest_dock_jpeg and time.time() - self.last_dock_img_time < 3.0),
                "has_hand_feed": bool(self.latest_hand_jpeg and time.time() - self.last_hand_img_time < 3.0),
            }

    # ── Điều khiển Tay Máy & Kẹp Chân Không từ Web ───────────────────────────
    def trigger_gripper(self, state: bool) -> Tuple[bool, str]:
        if not self.cli_gripper.wait_for_service(timeout_sec=1.5):
            return False, "Dịch vụ /gripper/switch không sẵn sàng"
        req = SetBool.Request()
        req.data = bool(state)
        self.cli_gripper.call_async(req)
        return True, "Đã gửi lệnh bật hút" if state else "Đã gửi lệnh nhả hút"

    def trigger_arm_pose(self, pose_name: str) -> Tuple[bool, str]:
        POSES = {
            "STOW": [0.160, -1.5708, -2.3562, 0.7854],
            "PRE_SLOT_HOVER": [0.160, -0.8697, 2.6281, -1.7584],
            "PRE_SLOT_1": [0.015, -0.8697, 2.6281, -1.7584],
            "OBSERVE_1": [0.015, -0.5796, 1.3609, -0.7813],
            "TOUCH_1": [0.015, -0.5796, 1.3609, -0.7813],
            "PRE_SLOT_2": [0.265, -0.8697, 2.6281, -1.7584],
            "OBSERVE_2": [0.265, -0.5796, 1.3609, -0.7813],
            "TOUCH_2": [0.265, -0.5796, 1.3609, -0.7813],
            "TRAY_HOVER": [0.160, -1.5860, -2.2536, 0.6980],
            "TRAY_TOUCH": [0.015, -1.5860, -2.2536, 0.6980],
            "PRE_SLOT_TIER_1": [0.015, -0.8697, 2.6281, -1.7584],
            "OBSERVE_TIER_1": [0.015, -0.5796, 1.3609, -0.7813],
            "TOUCH_TIER_1": [0.015, -0.5796, 1.3609, -0.7813],
            "PRE_SLOT_TIER_2": [0.265, -0.8697, 2.6281, -1.7584],
            "OBSERVE_TIER_2": [0.265, -0.5796, 1.3609, -0.7813],
            "TOUCH_TIER_2": [0.265, -0.5796, 1.3609, -0.7813],
            "STATION_IN_HOVER": [0.040, -0.5796, 1.3609, -0.7813],
            "STATION_OUT_HOVER": [0.040, -0.5796, 1.3609, -0.7813],
        }
        p_upper = pose_name.upper()
        if p_upper not in POSES:
            return False, f"Tư thế '{pose_name}' không hợp lệ"

        q = POSES[p_upper]
        traj = JointTrajectory()
        traj.joint_names = ["lift_joint", "shoulder_joint", "elbow_joint", "wrist_joint"]
        p = JointTrajectoryPoint()
        p.positions = q
        p.time_from_start.sec = 2
        traj.points.append(p)
        self.pub_arm_traj.publish(traj)
        return True, f"Đã gửi lệnh tay máy về tư thế {p_upper}"

    def clear_station_out(self) -> Tuple[bool, str]:
        """Gọi service /wh/clear_station_out để dọn sạch bàn xuất hàng."""
        if not self.cli_clear_out.wait_for_service(timeout_sec=1.5):
            return False, "Dịch vụ /wh/clear_station_out không sẵn sàng"
        req = Trigger.Request()
        fut = self.cli_clear_out.call_async(req)
        # Vì gọi từ async route FastAPI trong thread pool, ta có thể đợi một chút
        t0 = time.time()
        while not fut.done() and (time.time() - t0 < 3.0):
            time.sleep(0.05)
        if fut.done() and fut.result():
            return fut.result().success, fut.result().message
        return False, "Timeout khi chờ dịch vụ dọn bàn trạm OUT"

    def replenish_station_in(self) -> Tuple[bool, str]:
        """Gọi service /wh/replenish_station_in để tiếp thêm phôi vào trạm IN."""
        if not self.cli_replenish_in.wait_for_service(timeout_sec=1.5):
            return False, "Dịch vụ /wh/replenish_station_in không sẵn sàng"
        req = Trigger.Request()
        fut = self.cli_replenish_in.call_async(req)
        t0 = time.time()
        while not fut.done() and (time.time() - t0 < 3.0):
            time.sleep(0.05)
        if fut.done() and fut.result():
            return fut.result().success, fut.result().message
        return False, "Timeout khi chờ dịch vụ tiếp thêm phôi trạm IN"

