#!/usr/bin/env python3
"""test_data_integrity.py – Kiểm thử toàn vẹn dữ liệu kho và xử lý lỗi FSM (Bài 7.3).

Bộ kiểm thử sử dụng Mock Action Clients (chạy cực nhanh, không cần Gazebo) để kiểm tra:
1. Bảng `slots` và `transactions` không đổi sau khi nhiệm vụ FAILED (so sánh snapshot trước/sau).
2. Ô RESERVED luôn được giải phóng (không ô nào bị treo RESERVED).
3. Nhập sai mã hộp -> REJECT, trả hộp về bàn IN, tồn kho không đổi, ghi events WARN.
4. Không bao giờ có hai nhiệm vụ cùng giữ một ô.
5. Hai lệnh xuất cùng mã hàng đồng thời không chọn trùng ô (FIFO).
6. Tiêm lỗi (Fault Injection) trên >= 8 trạng thái khác nhau của FSM.
7. Xử lý E-stop an toàn và dừng khẩn cấp.
"""

import os
import pytest
import rclpy
from std_msgs.msg import Bool, Empty

from wh_task_manager.inventory_db import InventoryDB
from wh_task_manager.fsm import Event, State, FSMContext
from wh_task_manager.task_manager_node import TaskManagerNode


@pytest.fixture(scope="session", autouse=True)
def rclpy_init():
    """Khởi tạo ROS 2 context cho toàn bộ session kiểm thử."""
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def test_node():
    """Tạo instance TaskManagerNode với in-memory DB và mock actions."""
    node = TaskManagerNode(db_path=":memory:", mock_actions=True)
    db = node.db

    # Cấu hình trạng thái ô thử nghiệm: 2 ô chứa ITEM_20 (để test FIFO), các ô khác EMPTY
    with db.conn:
        db.conn.execute("UPDATE slots SET state = 'EMPTY', quantity = 0, item_code = NULL, stored_at = NULL, reserved_by_task = NULL;")
        db.conn.execute("UPDATE slots SET state = 'FULL', quantity = 1, item_code = 'ITEM_20', stored_at = '2026-10-01T08:00:00' WHERE slot_id = 'A-2-1';")
        db.conn.execute("UPDATE slots SET state = 'FULL', quantity = 1, item_code = 'ITEM_20', stored_at = '2026-10-02T10:00:00' WHERE slot_id = 'B-0-1';")

    yield node
    node.destroy_node()


def get_db_snapshot(db: InventoryDB):
    """Lấy ảnh chụp trạng thái hiện tại của bảng slots và transactions."""
    cur = db.conn.cursor()
    cur.execute("SELECT slot_id, state, quantity, item_code, reserved_by_task FROM slots ORDER BY slot_id;")
    slots_snap = {row[0]: (row[1], row[2], row[3], row[4]) for row in cur.fetchall()}

    cur.execute("SELECT task_id, type, item_code, slot_id, delta FROM transactions ORDER BY id;")
    tx_snap = [(row[0], row[1], row[2], row[3], row[4]) for row in cur.fetchall()]

    return slots_snap, tx_snap


# ── TEST 1: Baseline kiểm tra nhiệm vụ thành công ───────────────────────────
def test_successful_inbound_and_outbound(test_node):
    """Kiểm tra luồng chuẩn khi không có lỗi: Nhập thành công và Xuất thành công."""
    db = test_node.db

    # 1. Nhập ITEM_21
    tid_in = db.create_task("INBOUND", "ITEM_21", priority=1)
    db.start_task(tid_in)
    ctx_in = FSMContext(task_id=tid_in, task_type="INBOUND", item_code="ITEM_21")
    status_in = test_node.execute_task_fsm(ctx_in, step_delay=0.0)

    assert status_in == "SUCCEEDED"
    slot_in = db.get_slot(ctx_in.slot_id)
    assert slot_in["state"] == "FULL"
    assert slot_in["quantity"] == 1
    assert slot_in["item_code"] == "ITEM_21"
    assert slot_in["reserved_by_task"] is None

    # 2. Xuất ITEM_20
    tid_out = db.create_task("OUTBOUND", "ITEM_20", priority=1)
    db.start_task(tid_out)
    ctx_out = FSMContext(task_id=tid_out, task_type="OUTBOUND", item_code="ITEM_20")
    status_out = test_node.execute_task_fsm(ctx_out, step_delay=0.0)

    assert status_out == "SUCCEEDED"
    slot_out = db.get_slot(ctx_out.slot_id)
    assert slot_out["state"] == "EMPTY"
    assert slot_out["quantity"] == 0
    assert slot_out["item_code"] is None
    assert slot_out["reserved_by_task"] is None


# ── TEST 2: Tiêm lỗi (Fault Injection) trên >= 8 trạng thái khác nhau ────────
@pytest.mark.parametrize(
    "task_type, item_code, fault_state",
    [
        # 6 trạng thái lỗi trong chu trình OUTBOUND
        ("OUTBOUND", "ITEM_20", State.OUT_NAV_TO_SLOT),
        ("OUTBOUND", "ITEM_20", State.OUT_DOCK_SLOT),
        ("OUTBOUND", "ITEM_20", State.OUT_VERIFY_SLOT),
        ("OUTBOUND", "ITEM_20", State.OUT_PICK_FROM_SLOT),
        ("OUTBOUND", "ITEM_20", State.OUT_NAV_TO_OUT),
        ("OUTBOUND", "ITEM_20", State.OUT_PLACE_TO_STATION),
        # 6 trạng thái lỗi trong chu trình INBOUND
        ("INBOUND", "ITEM_21", State.IN_NAV_TO_IN),
        ("INBOUND", "ITEM_21", State.IN_DOCK_IN),
        ("INBOUND", "ITEM_21", State.IN_PICK_FROM_STATION),
        ("INBOUND", "ITEM_21", State.IN_NAV_TO_SLOT),
        ("INBOUND", "ITEM_21", State.IN_DOCK_SLOT),
        ("INBOUND", "ITEM_21", State.IN_PLACE_TO_SLOT),
    ],
)
def test_fault_injection_data_integrity(test_node, task_type, item_code, fault_state):
    """Tiêm lỗi tại từng trạng thái và kiểm tra bảo toàn dữ liệu tồn kho 100%."""
    db = test_node.db

    # Chụp ảnh DB trước khi thực thi
    slots_before, tx_before = get_db_snapshot(db)

    # Khởi tạo nhiệm vụ
    task_id = db.create_task(task_type, item_code, priority=1)
    db.start_task(task_id)
    ctx = FSMContext(task_id=task_id, task_type=task_type, item_code=item_code)

    # Chạy FSM với lỗi cố ý được tiêm tại fault_state
    status = test_node.execute_task_fsm(
        ctx,
        fault_state=fault_state,
        injected_event=Event.FAIL,
        step_delay=0.0,
    )

    # 1. Nhiệm vụ phải kết thúc ở trạng thái FAILED
    assert status == "FAILED"
    task_rec = db.get_task(task_id)
    assert task_rec["status"] == "FAILED"
    assert "FAULT_INJECTED" in task_rec["error"]

    # 2. Bảng slots và transactions KHÔNG ĐỔI sau khi nhiệm vụ FAILED
    slots_after, tx_after = get_db_snapshot(db)
    assert slots_after == slots_before, f"Bảng slots bị thay đổi khi lỗi ở {fault_state.value}!"
    assert tx_after == tx_before, f"Bảng transactions bị ghi sai khi lỗi ở {fault_state.value}!"

    # 3. Mọi ô RESERVED phải được giải phóng hoàn toàn
    for sid, (st, _, _, res_by) in slots_after.items():
        assert st != "RESERVED", f"Ô {sid} bị kẹt ở trạng thái RESERVED sau khi phục hồi!"
        assert res_by is None, f"Ô {sid} vẫn bị gán reserved_by_task={res_by}!"


# ── TEST 3: Nhập sai tag hộp -> REJECT ─────────────────────────────────────
def test_inbound_wrong_box_tag_rejects(test_node):
    """Kiểm tra phát hiện sai mã hộp khi nhập -> REJECT, trả hộp về bàn IN, ghi event WARN."""
    db = test_node.db
    slots_before, tx_before = get_db_snapshot(db)

    # Cố ý cấu hình tay gắp trả về tag hộp 99 (trong khi ITEM_21 mong đợi tag 21)
    test_node.mock_box_tag = 99

    task_id = db.create_task("INBOUND", "ITEM_21", priority=1)
    db.start_task(task_id)
    ctx = FSMContext(task_id=task_id, task_type="INBOUND", item_code="ITEM_21")

    status = test_node.execute_task_fsm(ctx, step_delay=0.0)

    # 1. Nhiệm vụ thất bại do REJECT
    assert status == "FAILED"
    task_rec = db.get_task(task_id)
    assert "BOX_TAG_MISMATCH" in task_rec["error"]

    # 2. Tồn kho slots và transactions hoàn toàn không đổi
    slots_after, tx_after = get_db_snapshot(db)
    assert slots_after == slots_before
    assert tx_after == tx_before

    # 3. Phải có bản ghi trong bảng events với mức WARN và nội dung REJECT
    cur = db.conn.cursor()
    cur.execute("SELECT level, message FROM events WHERE message LIKE '%REJECT%' AND level = 'WARN';")
    events = cur.fetchall()
    assert len(events) >= 1, "Không tìm thấy thông báo WARN REJECT trong bảng events!"

    # Khôi phục lại mock_box_tag
    test_node.mock_box_tag = None


# ── TEST 4: Không bao giờ có hai nhiệm vụ cùng giữ một ô ───────────────────
def test_no_two_tasks_hold_same_slot(test_node):
    """Kiểm tra không thể có 2 nhiệm vụ cùng đặt trước (RESERVED) một ô kệ."""
    db = test_node.db

    # Tạo 2 nhiệm vụ nhập
    t1 = db.create_task("INBOUND", "ITEM_21")
    t2 = db.create_task("INBOUND", "ITEM_22")

    s1, tag1, err1 = db.reserve_slot_for_inbound("ITEM_21", task_id=t1)
    s2, tag2, err2 = db.reserve_slot_for_inbound("ITEM_22", task_id=t2)

    assert err1 is None
    assert err2 is None
    assert s1 != s2, f"Hai nhiệm vụ cùng nhận được một ô: {s1}"

    # Kiểm tra trạng thái DB
    slot1_info = db.get_slot(s1)
    slot2_info = db.get_slot(s2)
    assert slot1_info["reserved_by_task"] == t1
    assert slot2_info["reserved_by_task"] == t2


# ── TEST 5: Hai lệnh xuất cùng mã hàng đồng thời không chọn trùng ô (FIFO) ──
def test_concurrent_outbound_fifo_different_slots(test_node):
    """Kiểm tra hai lệnh xuất cùng mã hàng chọn 2 ô khác nhau theo đúng thứ tự FIFO."""
    db = test_node.db

    # ITEM_20 đang có 2 ô: A-2-1 (stored 2026-10-01) và B-0-1 (stored 2026-10-02)
    t1 = db.create_task("OUTBOUND", "ITEM_20")
    t2 = db.create_task("OUTBOUND", "ITEM_20")

    s1, tag1, err1 = db.reserve_slot_for_outbound("ITEM_20", task_id=t1)
    s2, tag2, err2 = db.reserve_slot_for_outbound("ITEM_20", task_id=t2)

    assert err1 is None
    assert err2 is None
    assert s1 == "A-2-1", "Lệnh 1 phải chọn ô có stored_at sớm nhất (A-2-1)"
    assert s2 == "B-0-1", "Lệnh 2 phải chọn ô tiếp theo (B-0-1)"
    assert s1 != s2

    # Lệnh xuất thứ 3 phải báo OUT_OF_STOCK vì cả 2 ô đã được đặt
    t3 = db.create_task("OUTBOUND", "ITEM_20")
    s3, tag3, err3 = db.reserve_slot_for_outbound("ITEM_20", task_id=t3)
    assert err3 == "OUT_OF_STOCK"
    assert s3 is None


# ── TEST 6: Outbound lỗi không đặt lại được vào kệ -> giữ hộp trên khay ─────
def test_outbound_recovery_keep_on_tray_when_cannot_restore(test_node):
    """Kiểm tra khi gắp hộp ra mà không đặt lại được: giữ trên khay, ô nguồn vẫn ghi là FULL."""
    db = test_node.db

    task_id = db.create_task("OUTBOUND", "ITEM_20", priority=1)
    db.start_task(task_id)
    ctx = FSMContext(task_id=task_id, task_type="OUTBOUND", item_code="ITEM_20")

    # Giả lập lỗi khi đặt lại ô kệ (mock_place_fail = True)
    test_node.mock_place_fail = True

    status = test_node.execute_task_fsm(
        ctx,
        fault_state=State.OUT_PLACE_TO_STATION,
        injected_event=Event.FAIL,
        step_delay=0.0,
    )

    assert status == "FAILED"
    # Ô nguồn vẫn phải là FULL trong DB
    slot = db.get_slot(ctx.slot_id)
    assert slot["state"] == "FULL"
    assert slot["quantity"] == 1
    assert slot["item_code"] == "ITEM_20"
    assert slot["reserved_by_task"] is None

    # Phải có bản ghi WARN cần can thiệp vận hành
    cur = db.conn.cursor()
    cur.execute("SELECT level, message FROM events WHERE message LIKE '%khay xe, cần can thiệp%';")
    events = cur.fetchall()
    assert len(events) >= 1

    test_node.mock_place_fail = False


# ── TEST 7: E-stop dừng robot khẩn cấp ──────────────────────────────────────
def test_estop_freezes_and_fails_task_safely(test_node):
    """Kiểm tra kích hoạt E-stop dừng robot và đánh dấu nhiệm vụ FAILED an toàn."""
    db = test_node.db
    slots_before, tx_before = get_db_snapshot(db)

    # Kích hoạt E-stop
    test_node.cb_estop(Bool(data=True))
    assert test_node.estop_active is True

    # Tạo nhiệm vụ khi E-stop đang bật
    task_id = db.create_task("INBOUND", "ITEM_21", priority=1)
    db.start_task(task_id)
    ctx = FSMContext(task_id=task_id, task_type="INBOUND", item_code="ITEM_21")

    # Reset E-stop sau khi freeze
    def reset_later():
        test_node.cb_estop_reset(Empty())

    import threading
    t = threading.Timer(0.1, reset_later)
    t.start()

    status = test_node.execute_task_fsm(ctx, step_delay=0.05)
    t.join()

    assert status == "FAILED"
    slots_after, tx_after = get_db_snapshot(db)
    assert slots_after == slots_before
    assert tx_after == tx_before
