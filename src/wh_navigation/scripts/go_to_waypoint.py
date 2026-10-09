#!/usr/bin/env python3
"""go_to_waypoint.py – Điều hướng robot đến waypoint định danh trong kho hàng.

Bọc action NavigateToPose của Nav2:
- Tự động tra cứu tọa độ từ waypoints.yaml
- Xử lý thất bại: tự động thử lại 1 lần, nếu vẫn thất bại thực hiện xoay/lùi hồi phục
- Hỗ trợ chạy trực tiếp từ dòng lệnh:
    python3 go_to_waypoint.py --waypoint HOME
    python3 go_to_waypoint.py --waypoint SLOT_STAGING_3
- Hỗ trợ lắng nghe topic /wh/target_waypoint (std_msgs/String) để tích hợp task_manager.
"""

import argparse
import math
import os
import sys
import time
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String


def find_waypoints_file():
    # 1. Tra cứu từ package share directory của ROS 2 (chuẩn khi ros2 run)
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory("wh_navigation")
        candidate = os.path.join(share_dir, "config", "waypoints.yaml")
        if os.path.exists(candidate):
            return candidate
    except Exception:
        pass

    # 2. Tra cứu từ source tree
    curr = os.path.abspath(__file__)
    for _ in range(6):
        curr = os.path.dirname(curr)
        candidate = os.path.join(curr, "src", "wh_navigation", "config", "waypoints.yaml")
        if os.path.exists(candidate):
            return candidate
        candidate_ws = os.path.join(curr, "warehouse_ws", "src", "wh_navigation", "config", "waypoints.yaml")
        if os.path.exists(candidate_ws):
            return candidate_ws

    return "/home/lehuythien/warehouse_ws/src/wh_navigation/config/waypoints.yaml"


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__(
            "waypoint_navigator",
            parameter_overrides=[
                rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
            ]
        )
        self._action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.status_sub = self.create_subscription(String, "/wh/target_waypoint", self.target_wp_cb, 10)

        self.waypoints_file = find_waypoints_file()
        self.waypoints = self.load_waypoints()
        self._current_goal_handle = None
        self.is_navigating = False

        self.get_logger().info(f"WaypointNavigator khởi động. Đã nạp {len(self.waypoints)} waypoints từ {self.waypoints_file}.")

    def load_waypoints(self):
        if not os.path.exists(self.waypoints_file):
            self.get_logger().error(f"Không tìm thấy file: {self.waypoints_file}")
            return {}
        with open(self.waypoints_file, "r") as f:
            data = yaml.safe_load(f)
        return data.get("waypoints", {})

    def target_wp_cb(self, msg: String):
        wp_name = msg.data.strip()
        if self.is_navigating:
            self.get_logger().warn(f"Đang thực hiện hành trình khác. Hủy lệnh cũ để nhận lệnh mới: {wp_name}")
            self.cancel_current_goal()
        self.navigate_to(wp_name)

    def navigate_to(self, wp_name: str, max_retries: int = 1) -> bool:
        if wp_name not in self.waypoints:
            self.get_logger().error(f"Waypoint không tồn tại: '{wp_name}'. Danh sách: {list(self.waypoints.keys())}")
            return False

        wp = self.waypoints[wp_name]
        x, y, yaw = float(wp["x"]), float(wp["y"]), float(wp["yaw"])
        self.get_logger().info(f">>> Bắt đầu điều hướng đến {wp_name}: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}")

        for attempt in range(max_retries + 1):
            if attempt > 0:
                self.get_logger().warn(f"Thử lại lần {attempt} cho waypoint {wp_name} sau hồi phục...")
                self.execute_recovery()

            success = self._send_and_wait_goal(x, y, yaw)
            if success:
                self.get_logger().info(f"[THÀNH CÔNG] Robot đã đến đúng waypoint {wp_name}!")
                return True

        self.get_logger().error(f"[THẤT BẠI] Không thể đến waypoint {wp_name} sau {max_retries + 1} lần thử!")
        return False

    def _send_and_wait_goal(self, x: float, y: float, yaw: float) -> bool:
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Action server 'navigate_to_pose' chưa sẵn sàng!")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.is_navigating = True
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Mục tiêu bị từ chối bởi Nav2!")
            self.is_navigating = False
            return False

        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.is_navigating = False
        self._current_goal_handle = None

        status = result_future.result().status
        return status == GoalStatus.STATUS_SUCCEEDED

    def execute_recovery(self):
        """Thực hiện xoay nhẹ và lùi để giải phóng robot khi kẹt."""
        self.get_logger().info("Đang thực hiện hồi phục: lùi 0.5s và xoay 1.0s...")
        # Lùi nhẹ
        cmd = Twist()
        cmd.linear.x = -0.15
        for _ in range(10):
            self.cmd_pub.publish(cmd)
            time.sleep(0.05)
        # Xoay nhẹ
        cmd = Twist()
        cmd.angular.z = 0.4
        for _ in range(20):
            self.cmd_pub.publish(cmd)
            time.sleep(0.05)
        # Dừng
        self.cmd_pub.publish(Twist())
        time.sleep(0.2)

    def cancel_current_goal(self):
        if self._current_goal_handle is not None and self.is_navigating:
            self.get_logger().info("Đang gửi yêu cầu hủy mục tiêu hiện tại...")
            cancel_future = self._current_goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)


def main(args=None):
    parser = argparse.ArgumentParser(description="Điều hướng đến Waypoint kho hàng")
    parser.add_argument("--waypoint", type=str, default="", help="Tên waypoint (ví dụ: HOME, IN_STAGING, SLOT_STAGING_0)")
    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    navigator = WaypointNavigator()

    if cli_args.waypoint:
        success = navigator.navigate_to(cli_args.waypoint)
        navigator.destroy_node()
        rclpy.shutdown()
        sys.exit(0 if success else 1)
    else:
        navigator.get_logger().info("Chế độ dịch vụ: Đang chờ lệnh trên /wh/target_waypoint...")
        try:
            rclpy.spin(navigator)
        except KeyboardInterrupt:
            pass
        finally:
            navigator.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
