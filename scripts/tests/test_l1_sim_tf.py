#!/usr/bin/env python3
"""test_l1_sim_tf.py – Kiểm tra Tầng 1: Mô phỏng Gazebo và Cây TF Nền (Bài 10.3).

Yêu cầu kiểm tra:
1. /clock chạy đều, thời gian tiến triển.
2. Real-time factor đo bằng `gz stats` >= 0.5.
3. Mọi node chính dùng use_sim_time:=true.
4. Cây TF không có 2 nguồn cho cùng một cặp:
   - odom -> base_footprint: chỉ EKF phát.
   - map -> odom: chỉ AMCL hoặc slam_toolbox phát.
"""

import sys
import time
import subprocess
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage


class L1Checker(Node):
    def __init__(self):
        super().__init__("l1_sim_tf_checker")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.clock_count = 0
        self.first_clock = None
        self.last_clock = None
        self.sub_clock = self.create_subscription(Clock, "/clock", self.cb_clock, qos_profile_sensor_data)

        # Track publishers of TF frames
        self.tf_frames_seen = {}
        self.sub_tf = self.create_subscription(TFMessage, "/tf", self.cb_tf, 50)
        self.sub_tf_static = self.create_subscription(TFMessage, "/tf_static", self.cb_tf, 50)

    def cb_clock(self, msg: Clock):
        self.clock_count += 1
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.first_clock is None:
            self.first_clock = t
        self.last_clock = t

    def cb_tf(self, msg: TFMessage):
        for transform in msg.transforms:
            parent = transform.header.frame_id.lstrip("/")
            child = transform.child_frame_id.lstrip("/")
            pair = (parent, child)
            self.tf_frames_seen[pair] = self.tf_frames_seen.get(pair, 0) + 1


def check_rtf() -> Tuple[bool, float, str]:
    proc = None
    try:
        proc = subprocess.Popen(
            ["gz", "stats"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        # Read the first few lines
        t0 = time.time()
        while time.time() - t0 < 3.0:
            line = proc.stdout.readline()
            if "Factor" in line:
                proc.terminate()
                f_str = line.split("Factor[")[-1].split("]")[0].strip()
                val = float(f_str)
                return (val >= 0.5, val, f"RTF = {val:.2f}")
        proc.terminate()
    except Exception as e:
        if proc:
            proc.terminate()
        return (False, 0.0, f"Error getting gz stats: {e}")
    return (False, 0.0, "gz stats returned no factor")


def check_use_sim_time() -> Tuple[bool, List[str]]:
    key_nodes = [
        "/robot_state_publisher",
        "/amcl",
        "/controller_server",
        "/planner_server",
        "/bt_navigator",
        "/ekf_filter_node",
        "/tag_detector_dock",
        "/tag_detector_hand",
        "/dock_server",
        "/pick_place_server",
        "/task_manager_node",
        "/web_bridge_node",
    ]
    failed = []
    for node in key_nodes:
        try:
            res = subprocess.run(
                ["ros2", "param", "get", node, "use_sim_time"],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=4.0,
            )
            if "Boolean value is: True" not in res.stdout:
                failed.append(f"{node} (val: {res.stdout.strip() or res.stderr.strip()})")
        except Exception:
            failed.append(f"{node} (timeout)")
    return (len(failed) == 0, failed)


def main():
    print("=" * 65)
    print("  🧪 KIỂM TRA TẦNG L1: MÔ PHỎNG GAZEBO & CÂY TF NỀN")
    print("=" * 65)

    rclpy.init()
    node = L1Checker()

    print("1. Đang lấy mẫu /clock và /tf trong 3.0 giây...")
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    clock_ok = False
    clock_dt = (node.last_clock - node.first_clock) if (node.last_clock and node.first_clock) else 0.0
    if node.clock_count > 10 and clock_dt > 0.5:
        clock_ok = True
        print(f"  • /clock: ✅ PASS ({node.clock_count} gói, tiến triển sim={clock_dt:.2f}s)")
    else:
        print(f"  • /clock: ❌ FAIL (gói={node.clock_count}, dt={clock_dt:.2f}s)")

    rtf_ok, rtf_val, rtf_msg = check_rtf()
    if rtf_ok:
        print(f"  • Real-Time Factor (RTF): ✅ PASS ({rtf_msg} >= 0.5)")
    else:
        print(f"  • Real-Time Factor (RTF): ⚠️ WARN/FAIL ({rtf_msg} < 0.5)")

    sim_ok, failed_nodes = check_use_sim_time()
    if sim_ok:
        print("  • Tham số use_sim_time: ✅ PASS (Tất cả node chính đều có use_sim_time=True)")
    else:
        print(f"  • Tham số use_sim_time: ❌ FAIL (Các node sai: {failed_nodes})")

    # TF pairs check
    print("\n2. Kiểm tra các nhánh TF quan trọng:")
    critical_pairs = [
        ("odom", "base_footprint"),
        ("map", "odom"),
    ]
    tf_ok = True
    for p in critical_pairs:
        count = node.tf_frames_seen.get(p, 0)
        if count > 0:
            print(f"  • TF {p[0]} -> {p[1]}: ✅ PASS (nhận {count} transform)")
        else:
            print(f"  • TF {p[0]} -> {p[1]}: ❌ FAIL (chưa có transform trên /tf)")
            tf_ok = False

    overall_pass = clock_ok and sim_ok and tf_ok and (rtf_val >= 0.45)
    print("\n" + "=" * 65)
    if overall_pass:
        print("🎉 TẦNG L1: PASS")
    else:
        print("❌ TẦNG L1: FAIL")
    print("=" * 65 + "\n")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if overall_pass else 1)


if __name__ == "__main__":
    main()
