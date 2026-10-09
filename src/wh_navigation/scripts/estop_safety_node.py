#!/usr/bin/env python3
"""estop_safety_node.py – Node giám sát an toàn E-Stop khẩn cấp.

Khi nhận /estop = true:
- Ngay lập tức phát lệnh dừng /cmd_vel = 0 trong < 5ms.
- Duy trì phát lệnh 0 ở tần số 50 Hz cho đến khi mở khóa.
Khi nhận /estop = false:
- Giải phóng dòng lệnh, cho phép Nav2/teleop/servo tiếp tục.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class EstopSafetyNode(Node):
    def __init__(self):
        super().__init__("estop_safety_node")
        self.is_estopped = False

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd_muxed_pub = self.create_publisher(Twist, "/cmd_vel_muxed", 10)
        self.estop_sub = self.create_subscription(Bool, "/estop", self.estop_cb, 10)

        # Timer 50 Hz duy trì lệnh 0 khi estop = True
        self.timer = self.create_timer(0.02, self.timer_cb)
        self.get_logger().info("EstopSafetyNode khởi động và sẵn sàng bảo vệ an toàn.")

    def estop_cb(self, msg: Bool):
        self.is_estopped = bool(msg.data)
        if self.is_estopped:
            self.get_logger().warn("Khóa E-Stop được KÍCH HOẠT! Áp đặt /cmd_vel = 0 ngay lập tức.")
            zero_twist = Twist()
            self.cmd_pub.publish(zero_twist)
            self.cmd_muxed_pub.publish(zero_twist)
        else:
            self.get_logger().info("Khóa E-Stop đã MỞ! Khôi phục quyền điều khiển.")

    def timer_cb(self):
        if self.is_estopped:
            zero_twist = Twist()
            self.cmd_pub.publish(zero_twist)
            self.cmd_muxed_pub.publish(zero_twist)


def main(args=None):
    rclpy.init(args=args)
    node = EstopSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
