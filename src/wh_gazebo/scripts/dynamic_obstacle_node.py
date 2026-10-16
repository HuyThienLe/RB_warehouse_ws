#!/usr/bin/env python3
"""dynamic_obstacle_node.py – Điều khiển vật cản động (ô vuông nhỏ) tuần tra trong kho.

Vật cản là một khối hộp nhỏ (0.20m x 0.20m x 0.22m) màu cam nổi bật,
tuần tra qua lại trong hành lang kho:
  - x = -0.40 m (cắt ngang lộ trình giữa HOME/IN/OUT và các dãy kệ A/B)
  - y chạy liên tục từ -0.85 m đến +0.85 m với vận tốc 0.22 m/s
  - Dừng 0.8s ở mỗi đầu biên trước khi đảo chiều.

Kết hợp đồng thời 2 cơ chế chuyển động:
  1. Trực tiếp cập nhật Pose qua /gazebo/set_entity_state (100% không bị cản bởi ma sát sàn).
  2. Xuất lệnh vận tốc /dynamic_obstacle/cmd_vel cho plugin planar_move.
  3. Xuất /dynamic_obstacle/pose cho RViz và Web giám sát.
"""

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, PoseStamped, Vector3
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from std_srvs.srv import SetBool


class DynamicObstacleController(Node):
    def __init__(self):
        super().__init__("dynamic_obstacle_controller")

        # ── Tham số cấu hình ──────────────────────────────────────────────────
        self.declare_parameter("center_x", -0.40)      # Tọa độ x cố định (m)
        self.declare_parameter("patrol_min_y", -0.85)  # Biên âm trục y (m)
        self.declare_parameter("patrol_max_y", 0.85)   # Biên dương trục y (m)
        self.declare_parameter("speed", 0.22)          # Vận tốc tuần tra (m/s)
        self.declare_parameter("pause_time", 0.8)      # Dừng ở 2 biên (s)
        self.declare_parameter("height_z", 0.11)       # Tâm khối hộp theo z (m)
        self.declare_parameter("active", True)         # Trạng thái chạy

        self.center_x = float(self.get_parameter("center_x").value)
        self.min_y = float(self.get_parameter("patrol_min_y").value)
        self.max_y = float(self.get_parameter("patrol_max_y").value)
        self.speed = abs(float(self.get_parameter("speed").value))
        self.pause_time = float(self.get_parameter("pause_time").value)
        self.z = float(self.get_parameter("height_z").value)
        self.is_active = bool(self.get_parameter("active").value)

        # Tính toán chu kỳ tuần tra
        self.travel_distance = self.max_y - self.min_y
        self.travel_time = self.travel_distance / self.speed
        self.half_period = self.travel_time + self.pause_time
        self.period = 2.0 * self.half_period

        self.start_wall_time = time.time()
        self.last_log_time = time.time()

        # ── Publishers ────────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, "/dynamic_obstacle/cmd_vel", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/dynamic_obstacle/pose", 10)

        # ── Service Clients ───────────────────────────────────────────────────
        self.set_state_cli = self.create_client(SetEntityState, "/gazebo/set_entity_state")

        # Service toggle
        self.toggle_srv = self.create_service(SetBool, "~/toggle_patrol", self._handle_toggle)

        # Timer 20 Hz
        self.timer = self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f"Vật cản động (ô vuông nhỏ) sẵn sàng: x={self.center_x:.2f}m, "
            f"y in [{self.min_y:.2f}, {self.max_y:.2f}]m, v={self.speed:.2f}m/s, "
            f"chu kỳ={self.period:.1f}s"
        )

    def _handle_toggle(self, request, response):
        self.is_active = request.data
        status = "BẬT" if self.is_active else "TẮT"
        self.get_logger().info(f"Tuần tra vật cản động: {status}")
        response.success = True
        response.message = f"Vật cản: {status}"
        return response

    def _calculate_patrol(self, elapsed_t: float):
        """Tính toán vị trí y và vận tốc vy tại thời điểm elapsed_t."""
        if not self.is_active:
            return 0.0, 0.0

        t = elapsed_t % self.period

        if t < self.travel_time:
            # 1. Đi từ min_y -> max_y (hướng dương)
            y = self.min_y + self.speed * t
            vy = self.speed
        elif t < self.half_period:
            # 2. Dừng ở max_y
            y = self.max_y
            vy = 0.0
        elif t < self.half_period + self.travel_time:
            # 3. Đi từ max_y -> min_y (hướng âm)
            dt = t - self.half_period
            y = self.max_y - self.speed * dt
            vy = -self.speed
        else:
            # 4. Dừng ở min_y
            y = self.min_y
            vy = 0.0

        return y, vy

    def _tick(self):
        elapsed = time.time() - self.start_wall_time
        y, vy = self._calculate_patrol(elapsed)

        # 1. Gửi lệnh /cmd_vel cho planar_move
        cmd = Twist()
        cmd.linear.y = vy
        self.cmd_pub.publish(cmd)

        # 2. Đẩy Pose trực tiếp qua /gazebo/set_entity_state (không phụ thuộc ma sát)
        if self.set_state_cli.service_is_ready():
            req = SetEntityState.Request()
            req.state.name = "dynamic_obstacle"
            req.state.pose = Pose(
                position=Point(x=self.center_x, y=y, z=self.z),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            )
            req.state.twist = Twist(
                linear=Vector3(x=0.0, y=vy, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=0.0)
            )
            req.state.reference_frame = "world"
            self.set_state_cli.call_async(req)

        # 3. Xuất PoseStamped cho RViz
        p_msg = PoseStamped()
        p_msg.header.stamp = self.get_clock().now().to_msg()
        p_msg.header.frame_id = "map"
        p_msg.pose.position.x = self.center_x
        p_msg.pose.position.y = y
        p_msg.pose.position.z = self.z
        p_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(p_msg)

        # Ghi log định kỳ mỗi 8s để người dùng thấy rõ trạng thái
        if time.time() - self.last_log_time >= 8.0:
            self.last_log_time = time.time()
            self.get_logger().info(
                f"[Tuần tra] Vật cản đang tại: (x={self.center_x:.2f}, y={y:.2f}) m, vy={vy:.2f} m/s"
            )


def main(args=None):
    rclpy.init(args=args)
    node = DynamicObstacleController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
