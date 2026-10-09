#!/usr/bin/env python3
"""auto_mapper.py – Điều khiển robot đi quanh kho để quét laser và dựng bản đồ SLAM.

Bộ điều khiển P đơn giản dựa trên /odom_gt để di chuyển robot qua các điểm:
- Dọc tường kho
- Tiến vào lối đi giữa hai giá kệ (x in [0.5, 2.0], y = 0.0)
- Đi vòng ra lại và xoay tại các mốc để SLAM Toolbox quan sát đầy đủ 360 độ.
Tốc độ tối đa <= 0.2 m/s.
"""

import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class AutoMapper(Node):
    def __init__(self):
        super().__init__("auto_mapper")
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom_gt", self.odom_cb, 10)

        self.current_pose = None  # (x, y, yaw)

        # Danh sách điểm tuần tra lập bản đồ (x, y)
        self.waypoints = [
            (-2.2, 0.0),    # Xuất phát từ HOME
            (-2.2, 1.2),    # Lên trạm IN
            (-1.5, 1.4),    # Góc Tây Bắc
            (0.0, 1.4),     # Phía sau Kệ A
            (2.2, 1.4),     # Góc Đông Bắc
            (2.2, 0.0),     # Phía Đông lối đi
            (1.8, 0.0),     # Đi vào giữa hai giá kệ
            (0.8, 0.0),     # Giữa hai giá kệ
            (0.0, 0.0),     # Ra khỏi giá kệ
            (-1.5, -1.4),   # Góc Tây Nam
            (-2.2, -1.2),   # Trạm OUT
            (-2.2, 0.0),    # Về HOME
        ]

        self.timer = self.create_timer(0.05, self.control_loop)
        self.wp_index = 0
        self.state = "NAVIGATING"  # "NAVIGATING" hoặc "ROTATING"
        self.rot_start_time = 0.0
        self.rot_duration = 10.0   # Xoay 360 độ trong 10 giây (w = 0.63 rad/s)
        self.get_logger().info("AutoMapper node đã khởi động.")

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = (p.x, p.y, yaw)

    def control_loop(self):
        if self.current_pose is None:
            return

        cx, cy, cyaw = self.current_pose

        if self.wp_index >= len(self.waypoints):
            self.stop_robot()
            self.get_logger().info("Đã hoàn thành toàn bộ lộ trình lập bản đồ!")
            self.timer.cancel()
            return

        target_x, target_y = self.waypoints[self.wp_index]
        dx = target_x - cx
        dy = target_y - cy
        dist = math.hypot(dx, dy)

        if self.state == "NAVIGATING":
            if dist < 0.12:
                # Đã đến điểm, chuyển sang xoay quét laser
                self.state = "ROTATING"
                self.rot_start_time = time.time()
                self.get_logger().info(f"Đã đến waypoint {self.wp_index+1}/{len(self.waypoints)}: ({target_x:.2f}, {target_y:.2f}). Bắt đầu xoay quét...")
                return

            # Bộ điều khiển P holonomic
            target_yaw = math.atan2(dy, dx)
            yaw_err = math.atan2(math.sin(target_yaw - cyaw), math.cos(target_yaw - cyaw))

            cmd = Twist()
            # Giới hạn tốc độ tiến <= 0.18 m/s
            v_forward = min(0.18, max(0.05, 0.4 * dist))
            cmd.linear.x = v_forward
            cmd.angular.z = max(-0.5, min(0.5, 1.2 * yaw_err))
            self.cmd_pub.publish(cmd)

        elif self.state == "ROTATING":
            elapsed = time.time() - self.rot_start_time
            if elapsed < self.rot_duration:
                cmd = Twist()
                cmd.angular.z = 2.0 * math.pi / self.rot_duration  # Xoay đúng 1 vòng chậm
                self.cmd_pub.publish(cmd)
            else:
                self.stop_robot()
                self.state = "NAVIGATING"
                self.wp_index += 1

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = AutoMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
