#!/usr/bin/env python3
"""wait_ready.py – Chờ toàn bộ Action Server, Service, Topic và Web API sẵn sàng (Bài 09.1).

Kiểm tra:
  1. Topic: /robot_status, /map, /scan, /dock_camera/image_raw, /hand_camera/image_raw
  2. Service: /wh/submit_task, /wh/cancel_task (navigation mode)
  3. Action Server: /navigate_to_pose, /wh/dock_to_slot, /wh/pick_place (navigation mode)
  4. Web Portal: http://127.0.0.1:8000/api/mode

Thoát:
  - Mã 0: Hệ thống đã sẵn sàng 100%.
  - Mã 1: Quá thời gian chờ (Timeout).
"""

import sys
import time
import argparse
import urllib.request
import json

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data

from nav2_msgs.action import NavigateToPose
from wh_interfaces.action import DockToSlot, PickPlace
from wh_interfaces.srv import SubmitTask, CancelTask
from wh_interfaces.msg import RobotStatus
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, Image


class SystemReadinessChecker(Node):
    def __init__(self, mode="navigation"):
        super().__init__("system_readiness_checker")
        self.mode = mode

        # Biến theo dõi Topic
        self.seen_robot_status = False
        self.seen_map = False
        self.seen_scan = False
        self.seen_dock_cam = False
        self.seen_hand_cam = False

        qos_map = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

        # Subscribers
        self.create_subscription(RobotStatus, "/robot_status", lambda m: setattr(self, "seen_robot_status", True), 10)
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "seen_map", True), qos_map)
        self.create_subscription(LaserScan, "/scan", lambda m: setattr(self, "seen_scan", True), qos_profile_sensor_data)
        self.create_subscription(Image, "/dock_camera/image_raw", lambda m: setattr(self, "seen_dock_cam", True), qos_profile_sensor_data)
        self.create_subscription(Image, "/hand_camera/image_raw", lambda m: setattr(self, "seen_hand_cam", True), qos_profile_sensor_data)

        # Service Clients (navigation mode)
        if self.mode == "navigation":
            self.cli_submit = self.create_client(SubmitTask, "/wh/submit_task")
            self.cli_cancel = self.create_client(CancelTask, "/wh/cancel_task")

            # Action Clients
            self.act_nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")
            self.act_dock = ActionClient(self, DockToSlot, "/wh/dock_to_slot")
            self.act_pick = ActionClient(self, PickPlace, "/wh/pick_place")
        else:
            self.cli_submit = None
            self.cli_cancel = None
            self.act_nav = None
            self.act_dock = None
            self.act_pick = None

    def check_web(self) -> bool:
        try:
            req = urllib.request.Request("http://127.0.0.1:8000/api/mode", headers={"User-Agent": "WaitReadyChecker"})
            with urllib.request.urlopen(req, timeout=1.0) as resp:
                if resp.status == 200:
                    data = json.loads(resp.read().decode("utf-8"))
                    return "mode" in data
        except Exception:
            return False
        return False

    def is_all_ready(self) -> bool:
        # 1. Kiểm tra Topics
        topics_ok = self.seen_robot_status and self.seen_map and self.seen_scan
        if self.mode == "navigation":
            topics_ok = topics_ok and self.seen_dock_cam and self.seen_hand_cam

        # 2. Kiểm tra Services & Actions (chỉ ở navigation mode)
        if self.mode == "navigation":
            srv_submit_ok = self.cli_submit.service_is_ready()
            srv_cancel_ok = self.cli_cancel.service_is_ready()
            act_nav_ok = self.act_nav.server_is_ready()
            act_dock_ok = self.act_dock.server_is_ready()
            act_pick_ok = self.act_pick.server_is_ready()
            ros_nodes_ok = srv_submit_ok and srv_cancel_ok and act_nav_ok and act_dock_ok and act_pick_ok
        else:
            ros_nodes_ok = True

        # 3. Kiểm tra Web API
        web_ok = self.check_web()

        return topics_ok and ros_nodes_ok and web_ok

    def print_status_table(self):
        def icon(b): return "✅ Sẵn sàng" if b else "⏳ Đang chờ..."
        print("\n" + "-" * 60)
        print("  BẢNG KIỂM TRA TRẠNG THÁI KHỞI ĐỘNG HỆ THỐNG:")
        print(f"  • /robot_status:        {icon(self.seen_robot_status)}")
        print(f"  • /map:                 {icon(self.seen_map)}")
        print(f"  • /scan:                {icon(self.seen_scan)}")
        if self.mode == "navigation":
            print(f"  • /dock_camera/image:   {icon(self.seen_dock_cam)}")
            print(f"  • /hand_camera/image:   {icon(self.seen_hand_cam)}")
            print(f"  • /wh/submit_task:      {icon(self.cli_submit.service_is_ready())}")
            print(f"  • /wh/cancel_task:      {icon(self.cli_cancel.service_is_ready())}")
            print(f"  • /navigate_to_pose:    {icon(self.act_nav.server_is_ready())}")
            print(f"  • /wh/dock_to_slot:     {icon(self.act_dock.server_is_ready())}")
            print(f"  • /wh/pick_place:       {icon(self.act_pick.server_is_ready())}")
        print(f"  • Web Portal (Port 8000): {icon(self.check_web())}")
        print("-" * 60)


def main():
    parser = argparse.ArgumentParser(description="Chờ hệ thống AMR sẵn sàng hoàn toàn.")
    parser.add_argument("--timeout", type=float, default=60.0, help="Thời gian chờ tối đa (giây)")
    parser.add_argument("--mode", type=str, default="navigation", choices=["navigation", "mapping"], help="Chế độ vận hành")
    args = parser.parse_args()

    print("=" * 65)
    print(f"  🔍 BẮT ĐẦU KIỂM TRA TÍNH SẴN SÀNG HỆ THỐNG (Mode: {args.mode.upper()})")
    print(f"  ⏱️  Thời gian chờ tối đa: {args.timeout} giây")
    print("=" * 65)

    rclpy.init()
    checker = SystemReadinessChecker(mode=args.mode)

    start_time = time.time()
    last_print = 0.0

    try:
        while time.time() - start_time < args.timeout:
            rclpy.spin_once(checker, timeout_sec=0.2)

            if time.time() - last_print > 5.0:
                checker.print_status_table()
                last_print = time.time()

            if checker.is_all_ready():
                checker.print_status_table()
                elapsed = time.time() - start_time
                print(f"\n🎉 TOÀN BỘ HỆ THỐNG ĐÃ SẴN SÀNG SAU {elapsed:.2f} GIÂY!\n")
                checker.destroy_node()
                rclpy.shutdown()
                sys.exit(0)

            time.sleep(0.1)

        checker.print_status_table()
        print(f"\n❌ HẾT THỜI GIAN CHỜ ({args.timeout}s). Hệ thống chưa sẵn sàng đầy đủ!\n")
        checker.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    except KeyboardInterrupt:
        print("\nNgười dùng đã hủy kiểm tra.")
        checker.destroy_node()
        rclpy.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    main()
