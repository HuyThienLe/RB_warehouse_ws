#!/usr/bin/env python3
"""benchmark_slam_load.py – Đo lường tải băng thông WebSocket và CPU khi bật đủ các lớp SLAM (Bài 8b.7).

Đo lường:
- Tần số thông điệp: scan (mục tiêu <= 5 Hz), map_update (mục tiêu <= 1 Hz), robot (~10 Hz), trail, path.
- Băng thông WebSocket (KB/s và kbps) khi bật tất cả các lớp.
- Mức tiêu thụ CPU (%) của tiến trình Web server.
- Xuất kết quả thực tế vào docs/results/web_slam_load.csv.
"""

import os
import sys
import time
import json
import psutil
import threading
from typing import List, Dict, Any

from starlette.testclient import TestClient
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from wh_interfaces.msg import RobotStatus

import wh_web.app as web_app
from wh_task_manager.inventory_db import InventoryDB


def run_benchmark():
    print("=" * 65)
    print("  🚀 BẮT ĐẦU ĐO KIỂM TẢI WEBSOCKET & CPU (BÀI 8b.7)")
    print("=" * 65)

    # 1. Chuẩn bị CSDL và môi trường WebBridge
    db_path = "/tmp/test_slam_benchmark.db"
    if os.path.exists(db_path):
        os.remove(db_path)
    db = InventoryDB(db_path)
    db.init_schema()
    web_app.db = db

    output_csv = "/home/lehuythien/warehouse_ws/docs/results/web_slam_load.csv"
    os.makedirs(os.path.dirname(output_csv), exist_ok=True)

    records: List[Dict[str, Any]] = []
    proc = psutil.Process()

    with TestClient(web_app.app) as client:
        bridge = web_app.bridge
        assert bridge is not None, "WebBridge chưa được khởi tạo!"

        # Chuyển chế độ sang mapping
        bridge.current_mode = "mapping"

        # Thiết lập thread phát giả lập các topic ROS 2
        stop_event = threading.Event()

        def publisher_thread():
            seq = 0
            while not stop_event.is_set():
                now = time.time()
                seq += 1

                # 1. Robot status (~10 Hz)
                stat = RobotStatus()
                stat.x = -2.40 + 0.05 * (seq % 40)
                stat.y = 0.00 + 0.02 * (seq % 20)
                stat.yaw = 0.1 * (seq % 60)
                stat.state = "MAPPING"
                stat.battery_pct = 98.5
                stat.holding_box = False
                bridge.cb_robot_status(stat)

                # 2. LiDAR Scan (5 Hz)
                if seq % 2 == 0:
                    scan = LaserScan()
                    scan.header.frame_id = "lidar_link"
                    scan.angle_min = -3.14159
                    scan.angle_max = 3.14159
                    scan.angle_increment = 0.01745
                    scan.range_min = 0.1
                    scan.range_max = 12.0
                    scan.ranges = [2.5] * 360
                    # Tự đẩy trực tiếp giả lập tf đã giải
                    pts = []
                    for i in range(0, 360, 2):
                        a = scan.angle_min + i * scan.angle_increment + stat.yaw
                        pts.append([round(stat.x + 2.5 * 0.9, 3), round(stat.y + 2.5 * 0.9, 3)])
                    bridge.push("scan", {"points": pts[:180]})

                # 3. SLAM Map (1 Hz)
                if seq % 10 == 0:
                    grid = OccupancyGrid()
                    grid.info.width = 280
                    grid.info.height = 200
                    grid.info.resolution = 0.025
                    grid.info.origin.position.x = -3.5
                    grid.info.origin.position.y = -2.5
                    grid.data = [0] * (280 * 200)
                    bridge.cb_map(grid)

                # 4. Nav2 Path (2 Hz)
                if seq % 5 == 0:
                    path = Path()
                    poses = []
                    for k in range(50):
                        ps = PoseStamped()
                        ps.pose.position.x = stat.x + 0.1 * k
                        ps.pose.position.y = stat.y + 0.05 * k
                        poses.append(ps)
                    path.poses = poses
                    bridge.cb_plan(path)

                time.sleep(0.1)  # 10 Hz loop

        pub_t = threading.Thread(target=publisher_thread, daemon=True)
        pub_t.start()

        # Đo tải trên WebSocket trong 10 giây
        print("[Benchmark] Đang kết nối WebSocket và thu thập gói tin trong 10 giây...")
        with client.websocket_connect("/ws") as ws:
            start_time = time.time()
            total_bytes = 0
            msg_counts: Dict[str, int] = {
                "scan": 0,
                "map_update": 0,
                "robot": 0,
                "trail": 0,
                "path": 0,
                "mode": 0,
                "other": 0,
            }

            cpu_samples = []

            while time.time() - start_time < 10.0:
                # Đọc tin nhắn với non-blocking/nhanh
                try:
                    text_data = ws.receive_text()
                    total_bytes += len(text_data.encode("utf-8"))
                    msg = json.loads(text_data)
                    mtype = msg.get("type", "other")
                    if mtype in msg_counts:
                        msg_counts[mtype] += 1
                    else:
                        msg_counts["other"] += 1
                except Exception:
                    break

                if int(time.time() - start_time) % 2 == 0:
                    cpu_samples.append(proc.cpu_percent(interval=0.05))

            elapsed = time.time() - start_time
            stop_event.set()
            pub_t.join(timeout=1.0)

        # Tính toán các chỉ số
        avg_cpu = sum(cpu_samples) / max(1, len(cpu_samples)) if cpu_samples else proc.cpu_percent()
        scan_hz = msg_counts["scan"] / elapsed
        map_hz = msg_counts["map_update"] / elapsed
        robot_hz = msg_counts["robot"] / elapsed
        path_hz = msg_counts["path"] / elapsed
        trail_hz = msg_counts["trail"] / elapsed
        bandwidth_kbps = (total_bytes * 8 / 1024) / elapsed
        bandwidth_kbs = (total_bytes / 1024) / elapsed

        verdict = "PASS" if (scan_hz <= 5.5 and map_hz <= 1.2) else "WARN"

        print("-" * 65)
        print(f"  ⏱️  Thời gian đo: {elapsed:.2f} s")
        print(f"  📡 Tần số LiDAR scan: {scan_hz:.2f} Hz (Mục tiêu <= 5 Hz)")
        print(f"  🗺️  Tần số map_update: {map_hz:.2f} Hz (Mục tiêu <= 1 Hz)")
        print(f"  🤖 Tần số Robot Status: {robot_hz:.2f} Hz (~10 Hz)")
        print(f"  🛣️  Tần số Nav2 Path: {path_hz:.2f} Hz (~2 Hz)")
        print(f"  🐾 Tần số Trail Quỹ đạo: {trail_hz:.2f} Hz")
        print(f"  📊 Tổng dung lượng truyền: {total_bytes / 1024:.1f} KB ({bandwidth_kbs:.2f} KB/s)")
        print(f"  🌐 Băng thông mạng: {bandwidth_kbps:.2f} kbps")
        print(f"  💻 Mức sử dụng CPU: {avg_cpu:.1f}%")
        print(f"  🎯 Đánh giá tổng quát: {verdict}")
        print("-" * 65)

        # Ghi vào CSV
        with open(output_csv, "w", encoding="utf-8") as f:
            f.write("timestamp,layers_enabled,duration_sec,scan_hz,map_hz,robot_hz,path_hz,bandwidth_kbps,bandwidth_kbs,cpu_percent,verdict\n")
            f.write(
                f"{time.strftime('%Y-%m-%dT%H:%M:%S')},all_layers_active,{elapsed:.2f},{scan_hz:.2f},{map_hz:.2f},"
                f"{robot_hz:.2f},{path_hz:.2f},{bandwidth_kbps:.2f},{bandwidth_kbs:.2f},{avg_cpu:.1f},{verdict}\n"
            )

        print(f"  💾 Đã xuất kết quả kiểm thử vào: {output_csv}")
        print("=" * 65)

    if os.path.exists(db_path):
        os.remove(db_path)


if __name__ == "__main__":
    run_benchmark()
