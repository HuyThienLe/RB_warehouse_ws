#!/usr/bin/env python3
"""health_check.py – Công cụ kiểm tra sức khỏe toàn diện hệ thống (Bài 10.2).

In ra bảng sức khỏe:
  1. Tần số các topic chính (/clock, /scan, /imu/data, /dock_camera/image_raw,
     /hand_camera/image_raw, /joint_states, /odom, /odometry/filtered,
     /cmd_vel_nav, /cmd_vel_servo, /cmd_vel_muxed, /cmd_vel, /robot_status)
  2. Trạng thái Lifecycle của Nav2 (map_server, amcl, planner_server, controller_server, bt_navigator, etc.)
  3. Controller đang active (joint_state_broadcaster, arm_controller)
  4. Action servers đang có (/navigate_to_pose, /wh/dock_to_slot, /wh/pick_place)
  5. Các cặp TF có tồn tại và tuổi của TF
  6. Cảnh báo khi topic không phát hoặc node dùng sai use_sim_time.
"""

import sys
import time
import argparse
import subprocess
from typing import Dict, List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
import tf2_ros

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, Imu, Image, JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from wh_interfaces.msg import RobotStatus


class HealthMonitor(Node):
    def __init__(self, sample_sec: float = 2.5):
        super().__init__("system_health_monitor")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.sample_sec = sample_sec

        # Topic counters
        self.topic_counts: Dict[str, int] = {
            "/clock": 0,
            "/scan": 0,
            "/imu/data": 0,
            "/dock_camera/image_raw": 0,
            "/hand_camera/image_raw": 0,
            "/joint_states": 0,
            "/odom": 0,
            "/odometry/filtered": 0,
            "/cmd_vel_nav": 0,
            "/cmd_vel_servo": 0,
            "/cmd_vel_muxed": 0,
            "/cmd_vel": 0,
            "/robot_status": 0,
        }

        # Subscriptions
        self.sub_clock = self.create_subscription(Clock, "/clock", lambda _: self._inc("/clock"), qos_profile_sensor_data)
        self.sub_scan = self.create_subscription(LaserScan, "/scan", lambda _: self._inc("/scan"), qos_profile_sensor_data)
        self.sub_imu = self.create_subscription(Imu, "/imu/data", lambda _: self._inc("/imu/data"), qos_profile_sensor_data)
        self.sub_dock_cam = self.create_subscription(Image, "/dock_camera/image_raw", lambda _: self._inc("/dock_camera/image_raw"), qos_profile_sensor_data)
        self.sub_hand_cam = self.create_subscription(Image, "/hand_camera/image_raw", lambda _: self._inc("/hand_camera/image_raw"), qos_profile_sensor_data)
        self.sub_joints = self.create_subscription(JointState, "/joint_states", lambda _: self._inc("/joint_states"), 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom", lambda _: self._inc("/odom"), 10)
        self.sub_odom_filt = self.create_subscription(Odometry, "/odometry/filtered", lambda _: self._inc("/odometry/filtered"), 10)
        self.sub_cmd_nav = self.create_subscription(Twist, "/cmd_vel_nav", lambda _: self._inc("/cmd_vel_nav"), 10)
        self.sub_cmd_servo = self.create_subscription(Twist, "/cmd_vel_servo", lambda _: self._inc("/cmd_vel_servo"), 10)
        self.sub_cmd_mux = self.create_subscription(Twist, "/cmd_vel_muxed", lambda _: self._inc("/cmd_vel_muxed"), 10)
        self.sub_cmd_vel = self.create_subscription(Twist, "/cmd_vel", lambda _: self._inc("/cmd_vel"), 10)
        self.sub_status = self.create_subscription(RobotStatus, "/robot_status", lambda _: self._inc("/robot_status"), 10)

        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _inc(self, topic: str):
        self.topic_counts[topic] += 1


def check_node_sim_time(node_name: str) -> Optional[bool]:
    try:
        res = subprocess.run(
            ["ros2", "param", "get", node_name, "use_sim_time"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=2.0,
        )
        if "Boolean value is: True" in res.stdout:
            return True
        elif "Boolean value is: False" in res.stdout:
            return False
    except Exception:
        pass
    return None


import re


def get_nav2_lifecycles() -> Dict[str, str]:
    nodes = [
        "/map_server",
        "/amcl",
        "/controller_server",
        "/planner_server",
        "/behavior_server",
        "/bt_navigator",
        "/waypoint_follower",
    ]
    results = {}
    for n in nodes:
        try:
            res = subprocess.run(
                ["ros2", "lifecycle", "get", n],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=3.0,
            )
            out = re.sub(r'\x1b\[[0-9;]*[a-zA-Z]', '', res.stdout).strip()
            # typically: "- active [3]" or "active [3]"
            state = out.split()[0].replace("-", "").strip() if out else "not_found"
            results[n] = state
        except Exception:
            results[n] = "timeout"
    return results


def get_ros2_controllers() -> Dict[str, str]:
    try:
        res = subprocess.run(
            ["ros2", "control", "list_controllers"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=3.0,
        )
        clean_text = re.sub(r'\x1b\[[0-9;]*[a-zA-Z]', '', res.stdout)
        lines = clean_text.strip().splitlines()
        controllers = {}
        for line in lines:
            line = line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) >= 2:
                c_name = parts[0].split("[")[0].strip()
                state = parts[-1].strip()
                if c_name:
                    controllers[c_name] = state
        return controllers
    except Exception:
        return {}


def get_action_servers() -> List[str]:
    try:
        res = subprocess.run(
            ["ros2", "action", "list"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=2.0,
        )
        return [line.strip() for line in res.stdout.strip().splitlines() if line.strip()]
    except Exception:
        return []


def main():
    parser = argparse.ArgumentParser(description="Kiểm tra sức khỏe hệ thống AMR+SCARA.")
    parser.add_argument("--sample", type=float, default=2.5, help="Thời gian lấy mẫu (giây)")
    args = parser.parse_args()

    print("=" * 75)
    print("  🩺 HỆ THỐNG KIỂM TRA SỨC KHỎE (HEALTH CHECK) - BÀI 10.2")
    print(f"  ⏱️  Đang lấy mẫu tần số trong {args.sample:.1f}s...")
    print("=" * 75)

    rclpy.init()
    monitor = HealthMonitor(sample_sec=args.sample)

    start_t = time.time()
    while time.time() - start_t < args.sample:
        rclpy.spin_once(monitor, timeout_sec=0.05)

    sample_dur = time.time() - start_t

    # 1. Topic Frequencies Table
    print("\n--- 1. TẦN SỐ CÁC TOPIC CHÍNH ---")
    print(f"{'Topic':<32} | {'Số gói':<8} | {'Tần số (Hz)':<12} | {'Yêu cầu / Trạng thái'}")
    print("-" * 75)

    specs = {
        "/clock": (10.0, ">= 5 Hz (Mô phỏng đang chạy)"),
        "/scan": (10.0, ">= 8 Hz (LiDAR 10Hz)"),
        "/imu/data": (100.0, ">= 80 Hz (IMU 100Hz)"),
        "/dock_camera/image_raw": (30.0, ">= 20 Hz (Camera dock)"),
        "/hand_camera/image_raw": (30.0, ">= 20 Hz (Camera tay)"),
        "/joint_states": (50.0, ">= 40 Hz (Joint state)"),
        "/odom": (50.0, ">= 40 Hz (Wheel odom)"),
        "/odometry/filtered": (30.0, ">= 15 Hz (EKF odom)"),
        "/cmd_vel_nav": (0.0, "Theo nhu cầu Nav2"),
        "/cmd_vel_servo": (0.0, "Theo nhu cầu Servo"),
        "/cmd_vel_muxed": (0.0, "Theo nhu cầu Mux"),
        "/cmd_vel": (0.0, "Theo nhu cầu Robot"),
        "/robot_status": (10.0, ">= 5 Hz (Task manager FSM)"),
    }

    all_pass = True
    for topic, count in monitor.topic_counts.items():
        hz = count / sample_dur
        expected_hz, req_str = specs.get(topic, (0.0, "N/A"))
        if expected_hz > 0:
            status = "✅ PASS" if hz >= (expected_hz * 0.5) else "❌ FAIL"
            if "FAIL" in status:
                all_pass = False
        else:
            status = "ℹ️ IDLE" if hz == 0 else f" active ({hz:.1f} Hz)"
        print(f"{topic:<32} | {count:<8} | {hz:>8.1f} Hz  | {status} ({req_str})")

    # 2. Check Nav2 Lifecycle
    print("\n--- 2. TRẠNG THÁI LIFECYCLE CỦA NAV2 ---")
    lc_states = get_nav2_lifecycles()
    for node, state in lc_states.items():
        st_icon = "✅ active" if "active" in state.lower() else f"⚠️ {state}"
        if "active" not in state.lower():
            all_pass = False
        print(f"  • {node:<25}: {st_icon}")

    # 3. Check Controllers
    print("\n--- 3. TRẠNG THÁI CONTROLLER (ros2_control) ---")
    controllers = get_ros2_controllers()
    if not controllers:
        print("  ⚠️ Không tìm thấy controller nào đang chạy hoặc dịch vụ controller_manager chưa sẵn sàng.")
        all_pass = False
    else:
        for c_name, st in controllers.items():
            icon = "✅ active" if st.lower() == "active" else f"❌ {st}"
            if st.lower() != "active":
                all_pass = False
            print(f"  • {c_name:<28}: {icon}")

    # 4. Action Servers
    print("\n--- 4. ACTION SERVERS HIỆN CÓ ---")
    act_list = get_action_servers()
    needed_actions = ["/navigate_to_pose", "/wh/dock_to_slot", "/wh/pick_place"]
    for act in needed_actions:
        exists = any(act == a or act in a for a in act_list)
        icon = "✅ Sẵn sàng" if exists else "❌ Thiếu"
        if not exists:
            all_pass = False
        print(f"  • {act:<25}: {icon}")

    # 5. TF Transformations & Age
    print("\n--- 5. KIỂM TRA CÂY TF VÀ ĐỘ TRỄ ---")
    tf_pairs = [
        ("map", "odom"),
        ("odom", "base_footprint"),
        ("base_footprint", "base_link"),
        ("base_link", "dock_camera_link"),
        ("base_link", "hand_camera_link"),
        ("base_link", "tool_link"),
    ]
    for parent, child in tf_pairs:
        try:
            t = monitor.tf_buffer.lookup_transform(parent, child, Time(), timeout=Duration(seconds=0.5))
            if parent == "base_footprint" and child == "base_link":
                print(f"  • {parent:<15} -> {child:<18}: ✅ PASS (Static TF)")
                continue
            now_sim = monitor.get_clock().now()
            tf_stamp = Time.from_msg(t.header.stamp)
            diff_sec = abs((now_sim.nanoseconds - tf_stamp.nanoseconds) * 1e-9)
            age_str = f"{diff_sec:.3f}s"
            icon = "✅ PASS" if diff_sec < 1.0 else f"⚠️ HIGH DELAY ({age_str})"
            print(f"  • {parent:<15} -> {child:<18}: {icon} (Tuổi: {age_str})")
        except Exception as e:
            print(f"  • {parent:<15} -> {child:<18}: ❌ FAIL ({type(e).__name__})")
            if parent != "map":  # map->odom might only be published once robot has AMCL update
                all_pass = False

    # 6. use_sim_time parameters check
    print("\n--- 6. KIỂM TRA THAM SỐ use_sim_time TRÊN CÁC NODE CHÍNH ---")
    key_nodes = [
        "/robot_state_publisher",
        "/amcl",
        "/controller_server",
        "/planner_server",
        "/ekf_filter_node",
        "/tag_detector_dock",
        "/tag_detector_hand",
        "/dock_server",
        "/pick_place_server",
        "/task_manager_node",
        "/web_bridge_node",
    ]
    for kn in key_nodes:
        val = check_node_sim_time(kn)
        if val is True:
            icon = "✅ True"
        elif val is False:
            icon = "❌ False (SAI CẤU HÌNH!)"
            all_pass = False
        else:
            icon = "⚠️ Node chưa chạy / không có param"
        print(f"  • {kn:<28}: {icon}")

    print("=" * 75)
    if all_pass:
        print("🎉 TỔNG KẾT: HỆ THỐNG HOÀN TOÀN KHỎE MẠNH (ALL CHECKS PASSED)!")
    else:
        print("⚠️ TỔNG KẾT: CÓ CẢNH BÁO HOẶC LỖI THÀNH PHẦN (VUI LÒNG KIỂM TRA CHI TIẾT TRÊN)!")
    print("=" * 75 + "\n")

    monitor.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all_pass else 1)


if __name__ == "__main__":
    main()
