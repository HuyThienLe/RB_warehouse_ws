#!/usr/bin/env python3
"""test_api.py – Kiểm thử REST API và WebSocket của wh_web (Bài 8.6).

Kiểm tra:
1. Lệnh sai bị từ chối đúng mã HTTP (404, 409, 422).
2. Tạo nhiệm vụ thành công và hiển thị trong /api/tasks.
3. Hủy nhiệm vụ (/api/tasks/{id}/cancel).
4. E-stop và khôi phục (/api/estop, /api/estop/reset).
5. Bản đồ (/api/map, /api/map/image).
6. Kết nối WebSocket (/ws) nhận thông điệp robot, inventory, pong.
"""

import pytest
from starlette.testclient import TestClient

import wh_web.app as web_app
from wh_task_manager.inventory_db import InventoryDB


@pytest.fixture
def client(tmp_path):
    """Fixture tạo TestClient với CSDL cô lập cho mỗi bài test."""
    test_db_path = str(tmp_path / "test_inventory.db")
    test_db = InventoryDB(test_db_path)
    test_db.init_schema()

    # Nạp dữ liệu mẫu
    with test_db.conn:
        test_db.conn.executemany(
            """
            INSERT INTO items(item_code, name, box_tag_id, low_stock_threshold)
            VALUES (?, ?, ?, ?);
            """,
            [
                ("ITEM_20", "Bánh quy", 20, 1),
                ("ITEM_21", "Nước ngọt", 21, 1),
                ("ITEM_22", "Sữa tươi", 22, 1),
            ],
        )
        test_db.conn.executemany(
            """
            INSERT INTO slots(slot_id, rack, col, tier, slot_tag_id, state, quantity, item_code, stored_at)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?);
            """,
            [
                ("A-0-1", "rack_a", 0, 1, 0, "EMPTY", 0, None, None),
                ("A-1-1", "rack_a", 1, 1, 1, "EMPTY", 0, None, None),
                ("A-2-1", "rack_a", 2, 1, 2, "FULL", 1, "ITEM_20", "2026-10-01T08:00:00"),
                ("B-0-1", "rack_b", 0, 1, 6, "FULL", 1, "ITEM_20", "2026-10-02T10:00:00"),
            ],
        )

    # Gán DB test vào web_app
    web_app.db = test_db
    with TestClient(web_app.app) as test_client:
        web_app.db = test_db
        if web_app.bridge:
            web_app.bridge.db = test_db
        yield test_client

    test_db.close()
    web_app.db = None


# ── TEST 1: Danh mục & Tồn kho ──────────────────────────────────────────────
def test_get_inventory(client):
    res = client.get("/api/inventory")
    assert res.status_code == 200
    data = res.json()
    assert "items" in data
    assert "slot_stats" in data
    assert "slots_detail" in data
    assert data["slot_stats"]["full_count"] == 2
    assert data["slot_stats"]["empty_count"] == 2


def test_get_items(client):
    res = client.get("/api/items")
    assert res.status_code == 200
    items = res.json()
    assert len(items) >= 3
    item20 = next(i for i in items if i["item_code"] == "ITEM_20")
    assert item20["current_stock"] == 2
    assert item20["is_low_stock"] is False


# ── TEST 2: Kiểm thử Xác thực Lỗi (404, 409, 422) ───────────────────────────
def test_inbound_item_not_found_returns_404(client):
    """Mã hàng không tồn tại -> 404."""
    res = client.post("/api/tasks/inbound", json={"item_code": "NON_EXIST", "quantity": 1})
    assert res.status_code == 404
    assert "không tồn tại" in res.json()["detail"]


def test_inbound_quantity_out_of_bounds_returns_422(client):
    """Số lượng <= 0 hoặc > 10 -> 422."""
    res_zero = client.post("/api/tasks/inbound", json={"item_code": "ITEM_20", "quantity": 0})
    assert res_zero.status_code == 422

    res_too_large = client.post("/api/tasks/inbound", json={"item_code": "ITEM_20", "quantity": 11})
    assert res_too_large.status_code == 422


def test_inbound_no_free_slots_returns_409(client):
    """Không còn đủ ô trống khi nhập -> 409."""
    # Hiện tại chỉ có 2 ô trống, nếu yêu cầu 5 ô -> 409
    res = client.post("/api/tasks/inbound", json={"item_code": "ITEM_21", "quantity": 5})
    assert res.status_code == 409
    assert "NO_FREE_SLOT" in res.json()["detail"]


def test_outbound_out_of_stock_returns_409(client):
    """Xuất khi hết hàng -> 409 kèm lý do rõ ràng."""
    # ITEM_21 hiện có 0 hộp trong kho
    res = client.post("/api/tasks/outbound", json={"item_code": "ITEM_21", "quantity": 1})
    assert res.status_code == 409
    assert "OUT_OF_STOCK" in res.json()["detail"]


def test_outbound_insufficient_stock_returns_409(client):
    """Xuất nhiều hơn số lượng tồn -> 409."""
    # ITEM_20 có 2 hộp, yêu cầu xuất 3 hộp -> 409
    res = client.post("/api/tasks/outbound", json={"item_code": "ITEM_20", "quantity": 3})
    assert res.status_code == 409
    assert "OUT_OF_STOCK" in res.json()["detail"]


# ── TEST 3: Tạo và Hủy Nhiệm Vụ ─────────────────────────────────────────────
def test_create_and_list_tasks(client):
    """Tạo nhiệm vụ thành công và làm xuất hiện trong /api/tasks."""
    res_post = client.post("/api/tasks/inbound", json={"item_code": "ITEM_21", "quantity": 1})
    assert res_post.status_code == 200
    task_id = res_post.json()["task_id"]
    assert task_id > 0

    res_get = client.get("/api/tasks")
    assert res_get.status_code == 200
    tasks = res_get.json()
    assert any(t["task_id"] == task_id for t in tasks)


def test_cancel_task(client):
    """Hủy nhiệm vụ thành công."""
    res_post = client.post("/api/tasks/inbound", json={"item_code": "ITEM_21", "quantity": 1})
    task_id = res_post.json()["task_id"]

    res_cancel = client.post(f"/api/tasks/{task_id}/cancel")
    assert res_cancel.status_code == 200

    task_rec = web_app.db.get_task(task_id)
    assert task_rec["status"] == "CANCELLED"


# ── TEST 4: E-Stop và Khôi Phục ─────────────────────────────────────────────
def test_estop_and_reset(client):
    """Kích hoạt E-Stop và Khôi phục an toàn."""
    res_estop = client.post("/api/estop")
    assert res_estop.status_code == 200
    assert web_app.bridge.is_estopped is True

    res_robot = client.get("/api/robot")
    assert res_robot.json()["state"] == "ESTOP"

    res_reset = client.post("/api/estop/reset")
    assert res_reset.status_code == 200
    assert web_app.bridge.is_estopped is False


# ── TEST 5: Bản đồ Kho ──────────────────────────────────────────────────────
def test_map_endpoints(client):
    """Kiểm tra API metadata bản đồ và tải ảnh bản đồ."""
    res_meta = client.get("/api/map")
    assert res_meta.status_code == 200
    meta = res_meta.json()
    assert "resolution" in meta
    assert "origin" in meta
    assert meta["width"] == 280
    assert meta["height"] == 200

    res_img = client.get("/api/map/image")
    assert res_img.status_code == 200
    assert res_img.headers["content-type"] == "image/png"
    assert len(res_img.content) > 100


# ── TEST 6: WebSocket Streaming ─────────────────────────────────────────────
def test_websocket_stream(client):
    """Kiểm tra kết nối WebSocket nhận thông điệp robot, inventory, mode và phản hồi pong."""
    with client.websocket_connect("/ws") as websocket:
        received_types = set()
        for _ in range(3):
            msg = websocket.receive_json()
            received_types.add(msg["type"])
        assert any(t in received_types for t in ("mode", "robot", "inventory"))

        # Gửi ping nhận pong
        websocket.send_text("ping")
        pong = None
        for _ in range(5):
            m = websocket.receive_json()
            if m.get("type") == "pong":
                pong = m
                break
        assert pong == {"type": "pong"}


# ── TEST 7: Vision & Arm Manipulation Endpoints ──────────────────────────────
def test_vision_and_arm_endpoints(client):
    """Kiểm tra API thị giác và điều khiển cánh tay gắp trên Web."""
    # 1. Vision status
    res_vis = client.get("/api/vision/status")
    assert res_vis.status_code == 200
    vis_data = res_vis.json()
    assert "has_dock_feed" in vis_data
    assert "has_hand_feed" in vis_data
    assert "dock_tags" in vis_data
    assert "hand_tags" in vis_data

    # 2. Camera snapshots
    res_dock_snap = client.get("/api/vision/dock/live.jpg")
    assert res_dock_snap.status_code == 200
    assert "image/jpeg" in res_dock_snap.headers["content-type"]
    assert len(res_dock_snap.content) > 100

    res_hand_snap = client.get("/api/vision/hand/live.jpg")
    assert res_hand_snap.status_code == 200
    assert "image/jpeg" in res_hand_snap.headers["content-type"]
    assert len(res_hand_snap.content) > 100

    # 3. Arm gripper control
    res_grip_on = client.post("/api/arm/gripper", json={"state": True})
    assert res_grip_on.status_code == 200
    assert "success" in res_grip_on.json()

    res_grip_off = client.post("/api/arm/gripper", json={"state": False})
    assert res_grip_off.status_code == 200
    assert "success" in res_grip_off.json()

    # 4. Arm pose control
    res_pose = client.post("/api/arm/pose", json={"pose": "OBSERVE_TIER_1"})
    assert res_pose.status_code == 200
    assert "success" in res_pose.json()

