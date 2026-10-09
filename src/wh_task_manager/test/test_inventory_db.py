#!/usr/bin/env python3
"""test_inventory_db.py – Bộ kiểm thử CSDL tồn kho (Mục 7.1)."""

import os
import pytest
from wh_task_manager.inventory_db import InventoryDB


@pytest.fixture
def db():
    """Tạo CSDL in-memory cho mỗi test case."""
    database = InventoryDB(":memory:")
    database.init_schema()
    
    # Nạp mặt hàng mẫu
    with database.conn:
        database.conn.executemany(
            "INSERT INTO items(item_code, name, box_tag_id, low_stock_threshold) VALUES (?, ?, ?, ?);",
            [
                ("ITEM_20", "Bánh quy", 20, 1),
                ("ITEM_21", "Nước ngọt", 21, 1),
                ("ITEM_22", "Sữa chua", 22, 1),
            ]
        )
        # Nạp 4 ô kệ mẫu
        database.conn.executemany(
            "INSERT INTO slots(slot_id, rack, col, tier, slot_tag_id, state, quantity, item_code, stored_at) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?);",
            [
                ("A-0-1", "rack_a", 0, 1, 0, "EMPTY", 0, None, None),
                ("A-1-1", "rack_a", 1, 1, 1, "EMPTY", 0, None, None),
                ("A-0-2", "rack_a", 0, 2, 3, "EMPTY", 0, None, None),
                ("A-2-1", "rack_a", 2, 1, 2, "FULL", 1, "ITEM_20", "2026-10-01T10:00:00"),
                ("B-0-1", "rack_b", 0, 1, 6, "FULL", 1, "ITEM_20", "2026-10-02T10:00:00"),
            ]
        )
    yield database
    database.close()


def test_reserve_inbound_lowest_cost(db):
    """Kiểm tra chọn ô nhập có chi phí nhỏ nhất (khoảng cách + ưu tiên tầng 1)."""
    # IN_STAGING tại (-2.4, 0.45). Ô A-0-1 (col 0, tier 1, staging (0.75, 0.05)) gần hơn A-1-1 và tầng thấp hơn A-0-2
    slot_id, slot_tag, err = db.reserve_slot_for_inbound("ITEM_21", task_id=101)
    assert err is None
    assert slot_id == "A-0-1"
    assert slot_tag == 0

    slot = db.get_slot("A-0-1")
    assert slot["state"] == "RESERVED"
    assert slot["reserved_by_task"] == 101


def test_reserve_inbound_no_free_slot(db):
    """Kiểm tra báo lỗi NO_FREE_SLOT khi các ô đều bận."""
    with db.conn:
        db.conn.execute("UPDATE slots SET state = 'FULL' WHERE state = 'EMPTY';")

    slot_id, slot_tag, err = db.reserve_slot_for_inbound("ITEM_21", task_id=102)
    assert err == "NO_FREE_SLOT"
    assert slot_id is None


def test_reserve_inbound_item_not_found(db):
    """Kiểm tra báo lỗi khi mã hàng không tồn tại."""
    slot_id, slot_tag, err = db.reserve_slot_for_inbound("NON_EXISTING", task_id=103)
    assert err == "ITEM_NOT_FOUND"
    assert slot_id is None


def test_reserve_outbound_fifo(db):
    """Kiểm tra chọn ô xuất theo FIFO (stored_at sớm nhất)."""
    # ITEM_20 có ở A-2-1 (stored 2026-10-01) và B-0-1 (stored 2026-10-02)
    slot_id, slot_tag, err = db.reserve_slot_for_outbound("ITEM_20", task_id=201)
    assert err is None
    assert slot_id == "A-2-1"  # sớm nhất
    assert slot_tag == 2

    slot = db.get_slot("A-2-1")
    assert slot["state"] == "RESERVED"
    assert slot["reserved_by_task"] == 201


def test_reserve_outbound_out_of_stock(db):
    """Kiểm tra báo lỗi OUT_OF_STOCK khi mặt hàng không có trong kho."""
    slot_id, slot_tag, err = db.reserve_slot_for_outbound("ITEM_22", task_id=202)
    assert err == "OUT_OF_STOCK"
    assert slot_id is None


def test_release_slot_restores_state(db):
    """Kiểm tra giải phóng ô hoàn trả đúng trạng thái cũ."""
    # 1. Giải phóng ô nhập (trước đó EMPTY)
    slot_id, _, _ = db.reserve_slot_for_inbound("ITEM_21", task_id=301)
    assert db.get_slot(slot_id)["state"] == "RESERVED"
    db.release_slot(slot_id, task_id=301)
    assert db.get_slot(slot_id)["state"] == "EMPTY"
    assert db.get_slot(slot_id)["reserved_by_task"] is None

    # 2. Giải phóng ô xuất (trước đó FULL)
    slot_id, _, _ = db.reserve_slot_for_outbound("ITEM_20", task_id=302)
    assert db.get_slot(slot_id)["state"] == "RESERVED"
    db.release_slot(slot_id, task_id=302)
    assert db.get_slot(slot_id)["state"] == "FULL"
    assert db.get_slot(slot_id)["reserved_by_task"] is None


def test_commit_inbound_and_outbound(db):
    """Kiểm tra commit nhập và xuất ghi đúng slots và transactions."""
    # Nhập
    db.reserve_slot_for_inbound("ITEM_21", task_id=401)
    db.commit_inbound(task_id=401, slot_id="A-0-1", item_code="ITEM_21")
    slot = db.get_slot("A-0-1")
    assert slot["state"] == "FULL"
    assert slot["quantity"] == 1
    assert slot["item_code"] == "ITEM_21"
    assert slot["stored_at"] is not None

    txs = db.get_transactions()
    assert len(txs) >= 1
    assert txs[0]["delta"] == 1
    assert txs[0]["type"] == "INBOUND"

    # Xuất
    db.reserve_slot_for_outbound("ITEM_21", task_id=402)
    db.commit_outbound(task_id=402, slot_id="A-0-1", item_code="ITEM_21")
    slot = db.get_slot("A-0-1")
    assert slot["state"] == "EMPTY"
    assert slot["quantity"] == 0
    assert slot["item_code"] is None

    txs = db.get_transactions()
    assert txs[0]["delta"] == -1
    assert txs[0]["type"] == "OUTBOUND"


def test_inventory_summary(db):
    """Kiểm tra báo cáo tồn kho tổng hợp."""
    inv = db.get_inventory()
    assert "items" in inv
    assert "slot_stats" in inv
    assert inv["slot_stats"]["full_count"] == 2
    assert inv["slot_stats"]["empty_count"] == 3
