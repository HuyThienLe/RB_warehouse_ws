#!/usr/bin/env python3
"""test_localization.py – Kiểm tra định lượng Định vị và Bản đồ (Bài 10.5.7).

Yêu cầu (Mục 10.5.7):
- Cho robot đi 10 đoạn quanh kho.
- Ghi sai số map -> base_footprint so với ground truth theo thời gian.
- Tiêu chí:
  * Sai số vị trí trung bình < 5 cm (0.05 m).
  * Sai số vị trí lớn nhất < 10 cm (0.10 m).
  * Sai số hướng trung bình < 5.0 độ.
- Xuất CSV (docs/results/localization_test.csv) + biểu đồ (docs/results/localization_error.png).
"""

import sys
import math
import os
import time
import csv
import argparse
from typing import List, Tuple, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.action import ActionClient
import tf2_ros
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from gazebo_msgs.msg import ModelStates

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


class LocalizationTester(Node):
    def __init__(self):
        super().__init__(
            "localization_tester",
            parameter_overrides=[
                rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)
            ]
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.gt_pose = None  # (x, y, yaw)
        self.gt_history = []  # [(sim_time, x, y, yaw)]
        self.sub_model_states = self.create_subscription(
            ModelStates,
            "/gazebo/model_states",
            self.cb_model_states,
            10
        )

        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.log_data = []

    def cb_model_states(self, msg: ModelStates):
        if "warehouse_robot" in msg.name:
            idx = msg.name.index("warehouse_robot")
            p = msg.pose[idx].position
            q = msg.pose[idx].orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            self.gt_pose = (float(p.x), float(p.y), yaw)
            self.gt_history.append((now_sec, float(p.x), float(p.y), yaw))
            if len(self.gt_history) > 500:
                self.gt_history.pop(0)

    def get_amcl_pose(self) -> Tuple[float, float, float, float]:
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", Time(), Duration(seconds=0.5))
            x = float(t.transform.translation.x)
            y = float(t.transform.translation.y)
            q = t.transform.rotation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny, cosy)
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            return (x, y, yaw, now_sec)
        except Exception:
            return None

    def get_gt_at_time(self, tf_sec: float) -> Tuple[float, float, float]:
        if not self.gt_history:
            return self.gt_pose
        # Tìm mẫu GT có timestamp gần nhất với tf_sec
        best = min(self.gt_history, key=lambda item: abs(item[0] - tf_sec))
        if abs(best[0] - tf_sec) < 0.2:
            return (best[1], best[2], best[3])
        return self.gt_pose

    def send_goal(self, x: float, y: float, yaw: float, timeout_sec: float = 60.0) -> bool:
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose server không sẵn sàng!")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        fut = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or not fut.result().accepted:
            self.get_logger().error("Nav goal bị từ chối!")
            return False

        goal_handle = fut.result()
        res_fut = goal_handle.get_result_async()

        start_t = time.time()
        last_sample_t = 0.0
        while time.time() - start_t < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.05)

            # Lấy mẫu ở tần số ~10 Hz (cứ >= 0.1s)
            now_w = time.time()
            if now_w - last_sample_t >= 0.1:
                last_sample_t = now_w
                amcl_res = self.get_amcl_pose()
                if amcl_res:
                    ax, ay, ayaw, tf_sec = amcl_res
                    gt = self.get_gt_at_time(tf_sec)
                    if gt:
                        pos_err = math.hypot(ax - gt[0], ay - gt[1])
                        dyaw = math.atan2(math.sin(ayaw - gt[2]), math.cos(ayaw - gt[2]))
                        yaw_err_deg = abs(math.degrees(dyaw))
                        self.log_data.append({
                            "sim_time": round(tf_sec, 2),
                            "amcl_x": round(ax, 4),
                            "amcl_y": round(ay, 4),
                            "amcl_yaw": round(ayaw, 4),
                            "gt_x": round(gt[0], 4),
                            "gt_y": round(gt[1], 4),
                            "gt_yaw": round(gt[2], 4),
                            "pos_err_m": round(pos_err, 4),
                            "yaw_err_deg": round(yaw_err_deg, 3),
                        })

            if res_fut.done():
                status = res_fut.result().status
                return (status == GoalStatus.STATUS_SUCCEEDED)

        goal_handle.cancel_goal_async()
        return False


def main():
    parser = argparse.ArgumentParser(description="Kiểm tra định lượng sai số định vị AMCL.")
    parser.add_argument("--segments", type=int, default=10, help="Số đoạn hành trình di chuyển (mặc định 10)")
    args = parser.parse_args()

    print("=" * 68)
    print("  📍 BẮT ĐẦU KIỂM TRA ĐỊNH VỊ VÀ BẢN ĐỒ (MỤC 10.5.7)")
    print(f"  Số đoạn đường thử nghiệm: {args.segments}")
    print("=" * 68)

    rclpy.init()
    tester = LocalizationTester()

    # 10 waypoints bao phủ các khu vực kho
    waypoints = [
        ("IN_STAGING", -2.4, 0.45, 1.5708),
        ("SLOT_1_A", 1.25, 0.05, 1.5708),
        ("SLOT_0_A", 0.75, 0.05, 1.5708),
        ("SLOT_2_A", 1.75, 0.05, 1.5708),
        ("SLOT_1_B", 1.25, -0.05, -1.5708),
        ("SLOT_0_B", 0.75, -0.05, -1.5708),
        ("SLOT_2_B", 1.75, -0.05, -1.5708),
        ("OUT_STAGING", -2.4, -0.45, -1.5708),
        ("AISLE_MID", 0.0, 0.0, 0.0),
        ("HOME", -2.4, 0.0, 0.0),
    ]

    # Warmup and initial localization check
    time.sleep(1.0)
    for _ in range(15):
        rclpy.spin_once(tester, timeout_sec=0.1)

    # Đưa robot về HOME trước khi bắt đầu đo đạc chính thức
    if tester.gt_pose and math.hypot(tester.gt_pose[0] - (-2.4), tester.gt_pose[1] - 0.0) > 0.15:
        print("\nRobot đang ở vị trí khác, đưa robot về HOME trước khi đo đạc...")
        tester.send_goal(-2.4, 0.0, 0.0, timeout_sec=40.0)
        time.sleep(2.0)
        for _ in range(10):
            rclpy.spin_once(tester, timeout_sec=0.1)

    tester.log_data.clear()
    print("\nRobot đã ở vị trí HOME sẵn sàng. Bắt đầu đo đạc 10 đoạn hành trình...")

    completed_segments = 0
    for idx in range(min(args.segments, len(waypoints))):
        name, x, y, yaw = waypoints[idx]
        print(f"\n[{idx+1}/{args.segments}] Di chuyển đến {name} (x={x:.2f}, y={y:.2f}, yaw={yaw:.2f})...")
        t0 = time.time()
        ok = tester.send_goal(x, y, yaw, timeout_sec=50.0)
        dur = time.time() - t0
        res_str = "✅ ĐẾN NƠI" if ok else "⚠️ HẾT GIỜ/LỖI"
        if ok:
            completed_segments += 1
        print(f"  -> Kết quả: {res_str} ({dur:.1f}s)")
        time.sleep(1.0)

    # Thống kê số liệu
    results_dir = "/home/lehuythien/warehouse_ws/docs/results"
    os.makedirs(results_dir, exist_ok=True)
    csv_path = os.path.join(results_dir, "localization_test.csv")
    plot_path = os.path.join(results_dir, "localization_error.png")

    if tester.log_data:
        # Ghi CSV
        with open(csv_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=tester.log_data[0].keys())
            writer.writeheader()
            writer.writerows(tester.log_data)
        print(f"\n[OK] Đã lưu {len(tester.log_data)} mẫu đo vào: {csv_path}")

        pos_errors = [d["pos_err_m"] for d in tester.log_data]
        yaw_errors = [d["yaw_err_deg"] for d in tester.log_data]

        mean_pos_err = sum(pos_errors) / len(pos_errors)
        max_pos_err = max(pos_errors)
        mean_yaw_err = sum(yaw_errors) / len(yaw_errors)
        max_yaw_err = max(yaw_errors)

        pass_mean_pos = mean_pos_err < 0.05
        pass_max_pos = max_pos_err < 0.10
        pass_yaw = mean_yaw_err < 5.0
        all_passed = pass_mean_pos and pass_max_pos and pass_yaw

        print("\n" + "=" * 68)
        print("  📊 KẾT QUẢ ĐO LƯỜNG ĐỊNH VỊ (MỤC 10.5.7):")
        print(f"  • Sai số vị trí trung bình: {mean_pos_err*100:.2f} cm (Yêu cầu < 5.0 cm): {'✅ PASS' if pass_mean_pos else '❌ FAIL'}")
        print(f"  • Sai số vị trí lớn nhất  : {max_pos_err*100:.2f} cm (Yêu cầu < 10.0 cm): {'✅ PASS' if pass_max_pos else '❌ FAIL'}")
        print(f"  • Sai số góc trung bình   : {mean_yaw_err:.2f}° (Yêu cầu < 5.0°):      {'✅ PASS' if pass_yaw else '❌ FAIL'}")
        print(f"  • Sai số góc lớn nhất     : {max_yaw_err:.2f}°")
        print(f"  • Hoàn thành chặng đường  : {completed_segments}/{args.segments}")
        print(f"  • KẾT LUẬN TỔNG THỂ       : {'🏆 TẦNG L3 PASS' if all_passed else '❌ TẦNG L3 FAIL'}")
        print("=" * 68)

        # Vẽ biểu đồ
        times = [d["sim_time"] - tester.log_data[0]["sim_time"] for d in tester.log_data]
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True, dpi=150)
        ax1.plot(times, [e * 100 for e in pos_errors], 'b-', label='Sai số vị trí (cm)')
        ax1.axhline(5.0, color='g', linestyle='--', label='Ngưỡng trung bình 5cm')
        ax1.axhline(10.0, color='r', linestyle='--', label='Ngưỡng cực đại 10cm')
        ax1.set_ylabel('Sai số vị trí (cm)')
        ax1.set_title('Đánh giá sai số định vị AMCL map->base_footprint vs Ground Truth')
        ax1.grid(True, alpha=0.3)
        ax1.legend()

        ax2.plot(times, yaw_errors, 'm-', label='Sai số góc (°)')
        ax2.axhline(5.0, color='r', linestyle='--', label='Ngưỡng góc 5°')
        ax2.set_xlabel('Thời gian mô phỏng (s)')
        ax2.set_ylabel('Sai số góc (°)')
        ax2.grid(True, alpha=0.3)
        ax2.legend()

        plt.tight_layout()
        plt.savefig(plot_path)
        plt.close()
        print(f"[OK] Đã xuất biểu đồ định vị tại: {plot_path}")

        tester.destroy_node()
        rclpy.shutdown()
        sys.exit(0 if all_passed else 1)
    else:
        print("[ERROR] Không thu thập được dữ liệu định vị!")
        tester.destroy_node()
        rclpy.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    main()
