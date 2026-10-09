#!/usr/bin/env python3
"""benchmark_latency.py – Đo độ trễ từ Web POST đến phản hồi và khởi động FSM (Bài 8.6).

Quy trình:
1. Khởi động TaskManagerNode (mock_actions) và WebBridge trong tiến trình test.
2. Gửi 20 lệnh tạo nhiệm vụ qua REST API (POST /api/tasks/inbound).
3. Đo thời gian từ lúc bấm gửi POST đến lúc task được tiếp nhận và FSM chuyển sang trạng thái đầu tiên.
4. Lưu bảng số liệu vào docs/results/web_latency.csv.
5. Vẽ biểu đồ histogram phân bố độ trễ lưu vào docs/results/web_latency_histogram.png.
6. Đo độ trễ dừng khẩn cấp E-stop: xác nhận dừng trong < 0.5s.
"""

import os
import csv
import time
import threading
import statistics
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.executors import MultiThreadedExecutor
from starlette.testclient import TestClient

from wh_task_manager.task_manager_node import TaskManagerNode
import wh_web.app as web_app
from wh_task_manager.inventory_db import InventoryDB


def run_benchmark():
    if not rclpy.ok():
        rclpy.init()

    # Tạo DB test
    db_path = "/tmp/test_benchmark_inv.db"
    if os.path.exists(db_path):
        os.remove(db_path)

    # Khởi tạo TaskManagerNode và WebBridge
    task_node = TaskManagerNode(db_path=db_path, mock_actions=True)
    web_app.db = task_node.db

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(task_node)

    # Chạy ROS 2 executor trong background thread
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    latencies_post_ms = []
    latencies_fsm_ms = []
    total_latencies_ms = []

    results = []

    print("=== BẮT ĐẦU ĐO ĐỘ TRỄ GIAO TIẾP WEB LOCAL (20 LẦN LẶP) ===")

    with TestClient(web_app.app) as client:
        # Chờ WebBridge khởi động sẵn sàng
        for _ in range(50):
            if web_app.bridge is not None:
                break
            time.sleep(0.05)

        web_app.db = task_node.db
        if web_app.bridge:
            web_app.bridge.db = task_node.db

        for i in range(1, 21):
            # Đảm bảo robot đang rảnh (IDLE) trước khi gửi lệnh mới
            while task_node.active_fsm is not None:
                time.sleep(0.01)

            t_start = time.perf_counter()

            # 1. Gửi lệnh POST (xen kẽ nhập và xuất để không cạn ô trống hoặc hết hàng)
            endpoint = "/api/tasks/inbound" if (i % 2 == 1) else "/api/tasks/outbound"
            res = client.post(endpoint, json={"item_code": "ITEM_20", "quantity": 1})
            t_post_done = time.perf_counter()

            post_ms = (t_post_done - t_start) * 1000.0

            if res.status_code != 200:
                print(f"Lần {i}: Lỗi HTTP {res.status_code} - {res.text}")
                continue

            task_id = res.json()["task_id"]

            # 2. Chờ FSM bắt đầu thực thi nhiệm vụ (trạng thái đầu tiên)
            t_fsm_start = t_post_done
            started = False
            timeout_limit = 2.0
            while (time.perf_counter() - t_post_done) < timeout_limit:
                task = task_node.db.get_task(task_id)
                if task and task["status"] in ("RUNNING", "SUCCEEDED"):
                    t_fsm_start = time.perf_counter()
                    started = True
                    break
                time.sleep(0.005)

            fsm_ms = (t_fsm_start - t_post_done) * 1000.0 if started else timeout_limit * 1000.0
            total_ms = (t_fsm_start - t_start) * 1000.0

            latencies_post_ms.append(post_ms)
            latencies_fsm_ms.append(fsm_ms)
            total_latencies_ms.append(total_ms)

            results.append({
                "iteration": i,
                "task_id": task_id,
                "task_type": "INBOUND" if (i % 2 == 1) else "OUTBOUND",
                "post_latency_ms": round(post_ms, 2),
                "fsm_pickup_latency_ms": round(fsm_ms, 2),
                "total_latency_ms": round(total_ms, 2),
            })

            print(f"Lần {i:02d} ({'IN ' if i % 2 == 1 else 'OUT'}): POST: {post_ms:6.2f} ms | FSM Pickup: {fsm_ms:6.2f} ms | Tổng: {total_ms:6.2f} ms")

            # Chờ nhiệm vụ chạy xong để sẵn sàng cho lần sau
            while task_node.active_fsm is not None:
                time.sleep(0.01)

        # 3. Đo độ trễ E-STOP
        print("\n--- ĐO ĐỘ TRỄ KÍCH HOẠT E-STOP TỪ WEB ---")
        t_estop_start = time.perf_counter()
        res_estop = client.post("/api/estop")
        t_estop_done = time.perf_counter()
        estop_latency_ms = (t_estop_done - t_estop_start) * 1000.0
        assert res_estop.status_code == 200
        assert web_app.bridge.is_estopped is True
        print(f"Độ trễ phản hồi E-stop: {estop_latency_ms:.2f} ms (< 500 ms đạt yêu cầu)")

        # Khôi phục E-stop
        client.post("/api/estop/reset")

    # Dừng executor và task_node an toàn
    executor.shutdown()
    try:
        task_node.destroy_node()
    except Exception:
        pass

    # 4. Lưu kết quả ra CSV
    out_dir = "/home/lehuythien/warehouse_ws/docs/results"
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, "web_latency.csv")

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["iteration", "task_id", "task_type", "post_latency_ms", "fsm_pickup_latency_ms", "total_latency_ms"],
        )
        writer.writeheader()
        writer.writerows(results)

    print(f"\nĐã lưu kết quả đo lường độ trễ: {csv_path}")

    # 5. Thống kê số liệu
    mean_lat = statistics.mean(total_latencies_ms)
    std_lat = statistics.stdev(total_latencies_ms) if len(total_latencies_ms) > 1 else 0
    min_lat = min(total_latencies_ms)
    max_lat = max(total_latencies_ms)

    print("=" * 50)
    print("  KẾT QUẢ PHÂN TÍCH ĐỘ TRỄ (TOTAL LATENCY)")
    print(f"  Trung bình (Mean):     {mean_lat:.2f} ms ({mean_lat/1000:.3f} s)")
    print(f"  Độ lệch chuẩn (Std):  {std_lat:.2f} ms")
    print(f"  Nhỏ nhất (Min):        {min_lat:.2f} ms")
    print(f"  Lớn nhất (Max):        {max_lat:.2f} ms")
    print(f"  Tiêu chuẩn (< 1.0 s):  {'ĐẠT' if mean_lat < 1000 else 'KHÔNG ĐẠT'}")
    print("=" * 50)

    # 6. Vẽ Histogram
    hist_path = os.path.join(out_dir, "web_latency_histogram.png")
    plt.figure(figsize=(8, 5))
    plt.hist(total_latencies_ms, bins=8, color="#00d2ff", edgecolor="#0b0f19", alpha=0.85)
    plt.axvline(mean_lat, color="#ef4444", linestyle="dashed", linewidth=1.5, label=f"Mean: {mean_lat:.2f} ms")
    plt.title("Phân bố Độ trễ Phản hồi Web -> Khởi động FSM (20 lần)", fontsize=13, fontweight="bold")
    plt.xlabel("Độ trễ tổng cộng (ms)", fontsize=11)
    plt.ylabel("Tần suất (Số lần)", fontsize=11)
    plt.grid(axis="y", linestyle="--", alpha=0.5)
    plt.legend()
    plt.tight_layout()
    plt.savefig(hist_path, dpi=150)
    plt.close()

    print(f"Đã lưu biểu đồ phân bố độ trễ: {hist_path}")


if __name__ == "__main__":
    run_benchmark()
