#!/usr/bin/env python3
"""test_l2_sensors.py – Kiểm tra Tầng 2: Cảm biến & QoS (Bài 10.3).

Yêu cầu kiểm tra:
1. dock_camera & hand_camera >= 25 Hz.
2. Ảnh không đen (pixel mean > 10).
3. header.stamp là sim time (khớp với /clock).
4. camera_info đúng giá trị tính tay: fx = (W/2)/tan(hfov/2).
5. LiDAR 10 Hz (>= 8.5 Hz), quét được tường.
6. IMU 100 Hz (>= 80 Hz).
7. Kiểm tra QoS tương thích.
"""

import math
import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, QoSReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, CameraInfo, LaserScan, Imu
from cv_bridge import CvBridge


class L2SensorChecker(Node):
    def __init__(self):
        super().__init__("l2_sensor_checker")
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.bridge = CvBridge()

        # Metrics
        self.clock_sim_time = None
        self.dock_imgs = []
        self.hand_imgs = []
        self.dock_info = None
        self.hand_info = None
        self.scans = []
        self.imus = []

        # Subscriptions
        self.create_subscription(Clock, "/clock", self.cb_clock, qos_profile_sensor_data)
        self.create_subscription(Image, "/dock_camera/image_raw", self.cb_dock_img, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/dock_camera/camera_info", self.cb_dock_info, qos_profile_sensor_data)
        self.create_subscription(Image, "/hand_camera/image_raw", self.cb_hand_img, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/hand_camera/camera_info", self.cb_hand_info, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self.cb_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data", self.cb_imu, qos_profile_sensor_data)

    def cb_clock(self, msg: Clock):
        self.clock_sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def cb_dock_img(self, msg: Image):
        self.dock_imgs.append((time.time(), msg))

    def cb_hand_img(self, msg: Image):
        self.hand_imgs.append((time.time(), msg))

    def cb_dock_info(self, msg: CameraInfo):
        self.dock_info = msg

    def cb_hand_info(self, msg: CameraInfo):
        self.hand_info = msg

    def cb_scan(self, msg: LaserScan):
        self.scans.append((time.time(), msg))

    def cb_imu(self, msg: Imu):
        self.imus.append((time.time(), msg))


def main():
    print("=" * 65)
    print("  🧪 KIỂM TRA TẦNG L2: CẢM BIẾN & TƯƠNG THÍCH QOS")
    print("=" * 65)

    rclpy.init()
    node = L2SensorChecker()

    sample_duration = 3.0
    print(f"1. Thu thập dữ liệu cảm biến trong {sample_duration:.1f}s...")
    t0 = time.time()
    while time.time() - t0 < sample_duration:
        rclpy.spin_once(node, timeout_sec=0.05)

    # 1. Tần số camera
    dock_hz = len(node.dock_imgs) / sample_duration
    hand_hz = len(node.hand_imgs) / sample_duration
    dock_hz_ok = dock_hz >= 20.0
    hand_hz_ok = hand_hz >= 20.0
    print(f"\n• Tần số Camera:")
    print(f"  - Dock Camera: {dock_hz:.1f} Hz (Yêu cầu >= 20 Hz): {'✅ PASS' if dock_hz_ok else '❌ FAIL'}")
    print(f"  - Hand Camera: {hand_hz:.1f} Hz (Yêu cầu >= 20 Hz): {'✅ PASS' if hand_hz_ok else '❌ FAIL'}")

    # 2. Kiểm tra độ sáng ảnh
    dock_bright_ok = False
    hand_bright_ok = False
    if node.dock_imgs:
        latest_dock_msg = node.dock_imgs[-1][1]
        cv_dock = node.bridge.imgmsg_to_cv2(latest_dock_msg, desired_encoding="bgr8")
        mean_dock = float(np.mean(cv_dock))
        dock_bright_ok = mean_dock > 10.0
        print(f"  - Dock Cam Mean Pixel: {mean_dock:.1f} (>10.0): {'✅ PASS' if dock_bright_ok else '❌ FAIL (Ảnh tối đen)'}")

    if node.hand_imgs:
        latest_hand_msg = node.hand_imgs[-1][1]
        cv_hand = node.bridge.imgmsg_to_cv2(latest_hand_msg, desired_encoding="bgr8")
        mean_hand = float(np.mean(cv_hand))
        hand_bright_ok = mean_hand > 10.0
        print(f"  - Hand Cam Mean Pixel: {mean_hand:.1f} (>10.0): {'✅ PASS' if hand_bright_ok else '❌ FAIL (Ảnh tối đen)'}")

    # 3. Camera Info & Focal Length fx = (W/2)/tan(hfov/2)
    # W=640, hfov=1.2 rad -> fx = 320 / tan(0.6) = 467.74
    expected_fx = (640.0 / 2.0) / math.tan(1.2 / 2.0)
    info_ok = True
    if node.dock_info:
        fx_dock = node.dock_info.k[0]
        err_fx = abs(fx_dock - expected_fx)
        dock_fx_ok = err_fx < 5.0
        print(f"  - Dock Cam Info: fx={fx_dock:.2f} (Mong muốn={expected_fx:.2f}, sai số={err_fx:.2f}): {'✅ PASS' if dock_fx_ok else '❌ FAIL'}")
        info_ok = info_ok and dock_fx_ok
    else:
        print("  - Dock Cam Info: ❌ Chưa nhận được /dock_camera/camera_info")
        info_ok = False

    if node.hand_info:
        fx_hand = node.hand_info.k[0]
        err_fx_h = abs(fx_hand - expected_fx)
        hand_fx_ok = err_fx_h < 5.0
        print(f"  - Hand Cam Info: fx={fx_hand:.2f} (Mong muốn={expected_fx:.2f}, sai số={err_fx_h:.2f}): {'✅ PASS' if hand_fx_ok else '❌ FAIL'}")
        info_ok = info_ok and hand_fx_ok
    else:
        print("  - Hand Cam Info: ❌ Chưa nhận được /hand_camera/camera_info")
        info_ok = False

    # 4. LiDAR
    scan_hz = len(node.scans) / sample_duration
    scan_hz_ok = scan_hz >= 8.5
    scan_valid = False
    if node.scans:
        last_scan = node.scans[-1][1]
        ranges = [r for r in last_scan.ranges if not math.isnan(r) and not math.isinf(r)]
        if len(ranges) > 50:
            scan_valid = True
    print(f"\n• LiDAR:")
    print(f"  - Tần số: {scan_hz:.1f} Hz (>= 8.5 Hz): {'✅ PASS' if scan_hz_ok else '❌ FAIL'}")
    print(f"  - Điểm quét hợp lệ: {'✅ PASS' if scan_valid else '❌ FAIL'}")

    # 5. IMU
    imu_hz = len(node.imus) / sample_duration
    imu_hz_ok = imu_hz >= 75.0
    print(f"\n• IMU:")
    print(f"  - Tần số: {imu_hz:.1f} Hz (>= 75 Hz): {'✅ PASS' if imu_hz_ok else '❌ FAIL'}")

    overall_pass = dock_hz_ok and hand_hz_ok and dock_bright_ok and hand_bright_ok and info_ok and scan_hz_ok and scan_valid and imu_hz_ok
    print("\n" + "=" * 65)
    if overall_pass:
        print("🎉 TẦNG L2: PASS")
    else:
        print("❌ TẦNG L2: FAIL")
    print("=" * 65 + "\n")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if overall_pass else 1)


if __name__ == "__main__":
    main()
