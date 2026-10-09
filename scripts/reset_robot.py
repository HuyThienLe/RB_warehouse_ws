#!/usr/bin/env python3
"""reset_robot.py – Reset vị trí robot AMR về HOME và thu gọn tay máy."""

import sys
import time
import math
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():
    rclpy.init()
    node = Node("reset_robot_helper")

    set_cli = node.create_client(SetEntityState, "/gazebo/set_entity_state")
    pub_traj = node.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)

    print("Đang gỡ kẹt và reset robot AMR...")

    # 1. Đặt robot về trạm HOME ngay lập tức để giải phóng hoàn toàn va chạm vật lý với kệ
    if set_cli.wait_for_service(timeout_sec=3.0):
        req = SetEntityState.Request()
        req.state.name = "warehouse_robot"
        req.state.reference_frame = "world"
        req.state.pose.position.x = -2.4
        req.state.pose.position.y = 0.0
        req.state.pose.position.z = 0.02
        req.state.pose.orientation.z = 0.0
        req.state.pose.orientation.w = 1.0
        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = 0.0
        req.state.twist.angular.y = 0.0
        req.state.twist.angular.z = 0.0
        future = set_cli.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        print("-> Đã đưa robot về trạm HOME an toàn!")
    else:
        print("-> Không kết nối được Gazebo service, vui lòng kiểm tra simulation.")

    time.sleep(0.3)

    # 2. Thu gọn tay máy về STOW (d=0.0, q1=-2.60, q2=-2.40)
    msg = JointTrajectory()
    msg.joint_names = ["lift_joint", "shoulder_joint", "elbow_joint"]
    p = JointTrajectoryPoint()
    p.positions = [0.0, -2.60, -2.40]
    p.time_from_start.sec = 1
    msg.points.append(p)
    for _ in range(5):
        pub_traj.publish(msg)
        time.sleep(0.05)

    print("-> Đã đưa tay máy về tư thế gấp gọn STOW.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
