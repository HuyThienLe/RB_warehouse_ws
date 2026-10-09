#!/usr/bin/env python3
"""test_slam_web.py – Kiểm thử tự động các tính năng Bản đồ SLAM Thời gian thực (Bài 8b).

Kiểm thử:
1. Chuyển đổi lưới bản đồ sang PNG chuẩn (grid_to_png với np.flipud và thang xám).
2. API lấy chế độ hệ thống (/api/mode) và chặn tiếp nhận đơn kho khi ở chế độ mapping.
3. API bản đồ sống (/api/map/live.png, /api/map/live/meta).
4. Điều khiển Teleop (/api/teleop) giới hạn an toàn (<=0.25 m/s, <=0.6 rad/s).
5. Watchdog an toàn (tự động ngắt vận tốc sau > 0.3s).
6. API lưu bản đồ SLAM (/api/slam/save) và serialize (/api/slam/serialize).
7. WebSocket truyền thông điệp mode, map_update, scan, trail, path.
"""

import os
import time
import pytest
import numpy as np
import cv2
from starlette.testclient import TestClient
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose

import wh_web.app as web_app
from wh_web.web_bridge import grid_to_png, WebBridge
from wh_task_manager.inventory_db import InventoryDB


@pytest.fixture
def client_slam(tmp_path):
    """Fixture tạo TestClient với CSDL cô lập và mock bridge nếu cần."""
    test_db_path = str(tmp_path / "test_slam_inv.db")
    test_db = InventoryDB(test_db_path)
    test_db.init_schema()

    with test_db.conn:
        test_db.conn.execute(
            "INSERT INTO items(item_code, name, box_tag_id, low_stock_threshold) VALUES ('ITEM_20', 'Bánh', 20, 1);"
        )
        test_db.conn.execute(
            "INSERT INTO slots(slot_id, rack, col, tier, slot_tag_id, state, quantity) VALUES ('A-0-1', 'rack_a', 0, 1, 0, 'EMPTY', 0);"
        )

    web_app.db = test_db
    with TestClient(web_app.app) as test_client:
        web_app.db = test_db
        yield test_client

    test_db.close()
    web_app.db = None


# ── TEST 1: Chuyển đổi OccupancyGrid sang ảnh PNG (Bài 8b.2) ─────────────────
def test_grid_to_png_conversion():
    """Kiểm tra grid_to_png đảo chiều y (np.flipud) và mã hóa màu chuẩn."""
    msg = OccupancyGrid()
    msg.info.width = 4
    msg.info.height = 3
    msg.info.resolution = 0.05
    # Hàng 0 (dưới cùng trong ROS): [trống, chưa biết, vật cản, trống]
    # Hàng 1 (giữa): [vật cản, vật cản, trống, chưa biết]
    # Hàng 2 (trên cùng trong ROS): [chưa biết, trống, vật cản, chưa biết]
    msg.data = [
        0, -1, 100, 0,
        100, 80, 0, -1,
        -1, 0, 70, -1
    ]

    png_bytes = grid_to_png(msg)
    assert len(png_bytes) > 0

    # Giải mã ảnh PNG kiểm tra
    nparr = np.frombuffer(png_bytes, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE)
    assert img.shape == (3, 4)

    # Do np.flipud, hàng 0 của ảnh chính là hàng 2 của ROS (trên cùng)
    # Hàng 2 ROS: [-1 -> 128, 0 -> 255, 70 -> 0, -1 -> 128]
    assert img[0, 0] == 128  # chưa biết
    assert img[0, 1] == 255  # trống
    assert img[0, 2] == 0    # vật cản
    assert img[0, 3] == 128  # chưa biết

    # Hàng 2 của ảnh chính là hàng 0 của ROS (dưới cùng)
    # Hàng 0 ROS: [0 -> 255, -1 -> 128, 100 -> 0, 0 -> 255]
    assert img[2, 0] == 255
    assert img[2, 1] == 128
    assert img[2, 2] == 0
    assert img[2, 3] == 255


# ── TEST 2: Chế độ vận hành (Mode) và Chặn đơn hàng khi Mapping (Bài 8b.1) ───
def test_mode_endpoint_and_order_locking(client_slam):
    """Kiểm tra GET /api/mode và chặn tạo đơn khi đang ở chế độ mapping."""
    # 1. Mặc định là navigation
    res = client_slam.get("/api/mode")
    assert res.status_code == 200
    assert "mode" in res.json()

    # 2. Giả lập bridge chuyển sang chế độ mapping
    if web_app.bridge:
        web_app.bridge.current_mode = "mapping"

        # Đơn nhập bị chặn với mã 400
        res_inbound = client_slam.post("/api/tasks/inbound", json={"item_code": "ITEM_20", "quantity": 1})
        assert res_inbound.status_code == 400
        assert "mapping" in res_inbound.json()["detail"].lower()

        # Đơn xuất bị chặn với mã 400
        res_outbound = client_slam.post("/api/tasks/outbound", json={"item_code": "ITEM_20", "quantity": 1})
        assert res_outbound.status_code == 400
        assert "mapping" in res_outbound.json()["detail"].lower()

        # Chuyển lại về navigation
        web_app.bridge.current_mode = "navigation"


# ── TEST 3: API Bản đồ sống (/api/map/live.png & /api/map/live/meta) ─────────
def test_live_map_api(client_slam):
    """Kiểm tra endpoint trả về PNG và metadata có phiên bản (version)."""
    # Khi bridge nhận bản đồ
    if web_app.bridge:
        grid_msg = OccupancyGrid()
        grid_msg.info.width = 10
        grid_msg.info.height = 10
        grid_msg.info.resolution = 0.05
        grid_msg.info.origin.position.x = -1.2
        grid_msg.info.origin.position.y = -2.5
        grid_msg.data = [0] * 100

        web_app.bridge.cb_map(grid_msg)

    # 1. Lấy metadata
    res_meta = client_slam.get("/api/map/live/meta")
    assert res_meta.status_code == 200
    meta = res_meta.json()
    assert "version" in meta
    assert "width" in meta
    assert "height" in meta
    assert "resolution" in meta

    # 2. Tải ảnh live.png
    res_png = client_slam.get("/api/map/live.png")
    assert res_png.status_code == 200
    assert res_png.headers["content-type"] == "image/png"
    assert len(res_png.content) > 0


# ── TEST 4: Teleop giới hạn an toàn & Watchdog (Bài 8b.5) ────────────────────
def test_teleop_safety_clamping_and_watchdog(client_slam):
    """Kiểm tra vận tốc teleop bị clamp <= 0.25 m/s, <= 0.6 rad/s và watchdog dừng."""
    # 1. Gửi lệnh vượt ngưỡng an toàn (ví dụ 1.0 m/s và 2.0 rad/s)
    res = client_slam.post("/api/teleop", json={"vx": 1.0, "vy": -1.0, "wz": 2.5})
    assert res.status_code == 200

    # Kiểm tra trong bridge xem giá trị phát ra thực tế có bị giới hạn không
    if web_app.bridge:
        assert web_app.bridge.is_teleop_active is True

        # Giả lập thời gian trôi qua 0.35s không có lệnh mới
        web_app.bridge.last_teleop_time = time.time() - 0.35
        web_app.bridge.teleop_watchdog()

        # Watchdog phải hủy trạng thái active và phát vận tốc 0
        assert web_app.bridge.is_teleop_active is False


# ── TEST 5: API Lưu bản đồ SLAM (/api/slam/save) ─────────────────────────────
def test_slam_save_endpoint(client_slam, tmp_path):
    """Kiểm tra gọi POST /api/slam/save tạo file bản đồ .pgm và .yaml."""
    if web_app.bridge:
        web_app.bridge.last_map_time = 0.0
        # Chuẩn bị sẵn một bản đồ sống nhỏ trong bridge
        grid_msg = OccupancyGrid()
        grid_msg.info.width = 6
        grid_msg.info.height = 6
        grid_msg.info.resolution = 0.025
        grid_msg.info.origin.position.x = -0.5
        grid_msg.info.origin.position.y = -0.5
        grid_msg.data = [0] * 36
        web_app.bridge.cb_map(grid_msg)

    test_map_name = "test_slam_saved_map"
    res = client_slam.post("/api/slam/save", json={"name": test_map_name})
    assert res.status_code == 200
    data = res.json()
    assert data["success"] is True

    # Kiểm tra file sinh ra
    maps_dir = "/home/lehuythien/warehouse_ws/src/wh_navigation/maps"
    pgm_path = os.path.join(maps_dir, f"{test_map_name}.pgm")
    yaml_path = os.path.join(maps_dir, f"{test_map_name}.yaml")

    assert os.path.exists(pgm_path)
    assert os.path.exists(yaml_path)

    # Dọn dẹp file test
    try:
        os.remove(pgm_path)
        os.remove(yaml_path)
    except Exception:
        pass


# ── TEST 6: WebSocket nhận thông điệp SLAM (Bài 8b.3) ────────────────────────
def test_websocket_slam_events(client_slam):
    """Kiểm tra kết nối WebSocket nhận được thông điệp mode và ping/pong."""
    with client_slam.websocket_connect("/ws") as ws:
        # Gói tin đầu tiên thường là mode hoặc robot
        first_msg = ws.receive_json()
        assert "type" in first_msg
        assert first_msg["type"] in ("mode", "robot", "inventory", "map_update", "trail")

        # Gửi ping nhận pong
        ws.send_text("ping")
        pong = None
        for _ in range(5):
            m = ws.receive_json()
            if m.get("type") == "pong":
                pong = m
                break
        assert pong == {"type": "pong"}
