#!/usr/bin/env python3
"""inventory_db.py – Cơ sở dữ liệu tồn kho & nhiệm vụ kho hàng (SQLite).

Tuân thủ nghiêm ngặt Bài 7:
1. Schema chuẩn: items, slots, tasks, transactions, events.
2. Quy ước slot_id: A-<cột>-<tầng> hoặc B-<cột>-<tầng> (ví dụ A-0-1, A-2-2, B-1-2).
3. Nạp slots từ slots.yaml và items từ items_seed.yaml.
4. Mỗi hàm nghiệp vụ chạy trong một transaction nguyên tử (BEGIN IMMEDIATE).
5. Chi phí chọn ô nhập: Euclidean distance từ IN_STAGING đến SLOT_STAGING + 0.2 nếu tầng 2.
6. FIFO cho lệnh xuất (stored_at sớm nhất).
7. Báo lỗi sớm: ITEM_NOT_FOUND, OUT_OF_STOCK, NO_FREE_SLOT.
"""

import datetime
import math
import os
import sqlite3
import threading
from typing import Dict, List, Optional, Tuple, Any
import yaml


IN_STAGING_X = -2.40
IN_STAGING_Y = 0.45


def now_iso() -> str:
    """Trả về thời gian hiện tại chuẩn ISO-8601 string."""
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def slot_tag_to_slot_id(rack: str, col: int, tier: int) -> str:
    """Chuyển đổi quy ước định danh ô kệ sang chuẩn A-<cột>-<tầng> hoặc B-<cột>-<tầng>."""
    r = rack.lower().strip()
    rack_letter = "B" if ("rack_b" in r or r == "b" or r.endswith("_b")) else "A"
    return f"{rack_letter}-{col}-{tier}"


class InventoryDB:
    """Lớp giao tiếp CSDL SQLite cho kho hàng, hỗ trợ multi-threading và in-memory test."""

    def __init__(self, db_path: str = ":memory:"):
        self.db_path = db_path
        self._ensure_dir()
        self.lock = threading.RLock()
        self.conn = self._get_connection()
        self._setup_pragmas()

    def _ensure_dir(self):
        if self.db_path != ":memory:":
            dirname = os.path.dirname(os.path.abspath(self.db_path))
            if dirname:
                os.makedirs(dirname, exist_ok=True)

    def _get_connection(self) -> sqlite3.Connection:
        # timeout lớn để tránh lỗi database is locked
        conn = sqlite3.connect(self.db_path, timeout=30.0, check_same_thread=False)
        conn.row_factory = sqlite3.Row
        return conn

    def _setup_pragmas(self):
        with self.lock:
            with self.conn:
                self.conn.execute("PRAGMA foreign_keys = ON;")
                if self.db_path != ":memory:":
                    self.conn.execute("PRAGMA journal_mode = WAL;")

    def init_schema(self, schema_file: Optional[str] = None):
        """Khởi tạo cấu trúc bảng từ schema_file hoặc SQL mặc định."""
        if schema_file and os.path.exists(schema_file):
            with open(schema_file, "r", encoding="utf-8") as f:
                schema_sql = f.read()
        else:
            schema_sql = """
            CREATE TABLE IF NOT EXISTS items(
                item_code TEXT PRIMARY KEY,
                name TEXT,
                box_tag_id INTEGER UNIQUE,
                low_stock_threshold INTEGER DEFAULT 1
            );
            CREATE TABLE IF NOT EXISTS slots(
                slot_id TEXT PRIMARY KEY,
                rack TEXT,
                col INTEGER,
                tier INTEGER,
                slot_tag_id INTEGER UNIQUE,
                item_code TEXT REFERENCES items(item_code),
                quantity INTEGER DEFAULT 0,
                state TEXT DEFAULT 'EMPTY',
                reserved_by_task INTEGER,
                stored_at TEXT
            );
            CREATE TABLE IF NOT EXISTS tasks(
                task_id INTEGER PRIMARY KEY AUTOINCREMENT,
                type TEXT,
                item_code TEXT,
                slot_id TEXT,
                priority INTEGER DEFAULT 1,
                status TEXT DEFAULT 'QUEUED',
                created_at TEXT,
                started_at TEXT,
                finished_at TEXT,
                error TEXT
            );
            CREATE TABLE IF NOT EXISTS transactions(
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                task_id INTEGER,
                type TEXT,
                item_code TEXT,
                slot_id TEXT,
                delta INTEGER,
                ts TEXT
            );
            CREATE TABLE IF NOT EXISTS events(
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                level TEXT,
                message TEXT,
                ts TEXT
            );
            """
        with self.lock:
            with self.conn:
                self.conn.executescript(schema_sql)

    def load_seed_data(
        self,
        slots_file: Optional[str] = None,
        items_file: Optional[str] = None,
        inventory_seed_file: Optional[str] = None,
    ):
        """Nạp dữ liệu mẫu ban đầu cho slots, items và inventory ban đầu."""
        with self.lock:
            # 1. Nạp items từ items_seed.yaml
            if items_file and os.path.exists(items_file):
                with open(items_file, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f)
                    items = data.get("items", [])
                    with self.conn:
                        for item in items:
                            self.conn.execute(
                                """
                                INSERT OR IGNORE INTO items(item_code, name, box_tag_id, low_stock_threshold)
                                VALUES (?, ?, ?, ?);
                                """,
                                (
                                    item["item_code"],
                                    item.get("name", item["item_code"]),
                                    item["box_tag_id"],
                                    item.get("low_stock_threshold", 1),
                                ),
                            )

            # 2. Nạp slots từ slots.yaml
            if slots_file and os.path.exists(slots_file):
                with open(slots_file, "r", encoding="utf-8") as f:
                    slots_data = yaml.safe_load(f).get("slots", {})
                    with self.conn:
                        for tag_id, s in slots_data.items():
                            tag_id = int(tag_id)
                            rack = s["rack"]
                            col = int(s["col"])
                            tier = int(s["tier"])
                            sid = slot_tag_to_slot_id(rack, col, tier)
                            self.conn.execute(
                                """
                                INSERT OR IGNORE INTO slots(slot_id, rack, col, tier, slot_tag_id, state, quantity)
                                VALUES (?, ?, ?, ?, ?, 'EMPTY', 0);
                                """,
                                (sid, rack, col, tier, tag_id),
                            )

            # 3. Nạp tồn kho sẵn có từ inventory_seed.yaml
            if inventory_seed_file and os.path.exists(inventory_seed_file):
                with open(inventory_seed_file, "r", encoding="utf-8") as f:
                    inv_data = yaml.safe_load(f).get("boxes", [])
                    with self.conn:
                        for box in inv_data:
                            box_id = box["box_id"]
                            slot_tag = box.get("slot_id")
                            if slot_tag is not None:
                                # Tìm item_code tương ứng với box_id
                                cur = self.conn.execute(
                                    "SELECT item_code FROM items WHERE box_tag_id = ?;", (box_id,)
                                )
                                row = cur.fetchone()
                                item_code = row["item_code"] if row else f"ITEM_{box_id}"
                                # Đảm bảo item tồn tại
                                self.conn.execute(
                                    "INSERT OR IGNORE INTO items(item_code, name, box_tag_id) VALUES (?, ?, ?);",
                                    (item_code, f"Mặt hàng {box_id}", box_id),
                                )
                                # Cập nhật slot thành FULL
                                ts = now_iso()
                                self.conn.execute(
                                    """
                                    UPDATE slots
                                    SET state = 'FULL', quantity = 1, item_code = ?, stored_at = ?
                                    WHERE slot_tag_id = ?;
                                    """,
                                    (item_code, ts, slot_tag),
                                )

    # ── Nghiệp vụ Đặt ô & Quản lý Tồn kho ────────────────────────────────────

    def reserve_slot_for_inbound(
        self, item_code: str, task_id: int
    ) -> Tuple[Optional[str], Optional[int], Optional[str]]:
        """Chọn ô EMPTY chi phí nhỏ nhất, đặt RESERVED trong 1 transaction nguyên tử.
        
        Chi phí = khoảng cách Euclid từ IN_STAGING (-2.4, 0.45) đến SLOT_STAGING + 0.2 nếu tầng 2.
        Trả về: (slot_id, slot_tag_id, error_code).
        """
        with self.lock:
            cur = self.conn.cursor()
            cur.execute("BEGIN IMMEDIATE;")
            try:
                # 1. Kiểm tra mã hàng có tồn tại không
                cur.execute("SELECT item_code FROM items WHERE item_code = ?;", (item_code,))
                if not cur.fetchone():
                    self.conn.rollback()
                    self.add_event("WARN", f"Nhiệm vụ {task_id}: Không tìm thấy mã hàng '{item_code}'")
                    return None, None, "ITEM_NOT_FOUND"

                # 2. Tìm tất cả các ô đang EMPTY
                cur.execute(
                    """
                    SELECT slot_id, rack, col, tier, slot_tag_id
                    FROM slots
                    WHERE state = 'EMPTY';
                    """
                )
                empty_slots = cur.fetchall()
                if not empty_slots:
                    self.conn.rollback()
                    self.add_event("WARN", f"Nhiệm vụ {task_id}: Hết ô trống (NO_FREE_SLOT)")
                    return None, None, "NO_FREE_SLOT"

                # 3. Tính chi phí cho từng ô
                best_slot = None
                min_cost = float("inf")

                for s in empty_slots:
                    # Tọa độ staging của ô:
                    # rack_a: y = 0.05, rack_b: y = -0.05
                    # col 0: x = 0.75, col 1: x = 1.25, col 2: x = 1.75
                    col_x = 0.75 + s["col"] * 0.50
                    r = s["rack"].lower().strip()
                    staging_y = -0.05 if ("rack_b" in r or r == "b" or r.endswith("_b")) else 0.05
                    dist = math.sqrt((col_x - IN_STAGING_X) ** 2 + (staging_y - IN_STAGING_Y) ** 2)
                    tier_cost = 0.2 if s["tier"] == 2 else 0.0
                    cost = dist + tier_cost

                    if cost < min_cost:
                        min_cost = cost
                        best_slot = s

                slot_id = best_slot["slot_id"]
                slot_tag_id = best_slot["slot_tag_id"]

                # 4. Cập nhật trạng thái RESERVED
                cur.execute(
                    """
                    UPDATE slots
                    SET state = 'RESERVED', reserved_by_task = ?
                    WHERE slot_id = ?;
                    """,
                    (task_id, slot_id),
                )
                self.conn.commit()
                self.add_event(
                    "INFO",
                    f"Nhiệm vụ {task_id}: Đã đặt trước ô nhập {slot_id} (tag {slot_tag_id}, chi phí={min_cost:.2f}m)",
                )
                return slot_id, slot_tag_id, None
            except Exception as e:
                self.conn.rollback()
                self.add_event("ERROR", f"Lỗi transaction reserve_slot_for_inbound: {e}")
                raise

    def reserve_slot_for_outbound(
        self, item_code: str, task_id: int
    ) -> Tuple[Optional[str], Optional[int], Optional[str]]:
        """Chọn ô FULL của mã hàng đó có stored_at sớm nhất (FIFO), đặt RESERVED.
        
        Trả về: (slot_id, slot_tag_id, error_code).
        """
        with self.lock:
            cur = self.conn.cursor()
            cur.execute("BEGIN IMMEDIATE;")
            try:
                # 1. Kiểm tra mã hàng
                cur.execute("SELECT item_code FROM items WHERE item_code = ?;", (item_code,))
                if not cur.fetchone():
                    self.conn.rollback()
                    self.add_event("WARN", f"Nhiệm vụ {task_id}: Không tìm thấy mã hàng '{item_code}'")
                    return None, None, "ITEM_NOT_FOUND"

                # 2. Tìm ô FULL của mã hàng, sắp xếp FIFO theo stored_at
                cur.execute(
                    """
                    SELECT slot_id, slot_tag_id, stored_at
                    FROM slots
                    WHERE state = 'FULL' AND item_code = ?
                    ORDER BY stored_at ASC, slot_id ASC
                    LIMIT 1;
                    """,
                    (item_code,),
                )
                row = cur.fetchone()
                if not row:
                    self.conn.rollback()
                    self.add_event("WARN", f"Nhiệm vụ {task_id}: Hết hàng mã '{item_code}' (OUT_OF_STOCK)")
                    return None, None, "OUT_OF_STOCK"

                slot_id = row["slot_id"]
                slot_tag_id = row["slot_tag_id"]

                # 3. Đặt trạng thái RESERVED
                cur.execute(
                    """
                    UPDATE slots
                    SET state = 'RESERVED', reserved_by_task = ?
                    WHERE slot_id = ?;
                    """,
                    (task_id, slot_id),
                )
                self.conn.commit()
                self.add_event(
                    "INFO",
                    f"Nhiệm vụ {task_id}: Đã đặt trước ô xuất {slot_id} (tag {slot_tag_id}, FIFO stored_at={row['stored_at']})",
                )
                return slot_id, slot_tag_id, None
            except Exception as e:
                self.conn.rollback()
                self.add_event("ERROR", f"Lỗi transaction reserve_slot_for_outbound: {e}")
                raise

    def release_slot(self, slot_id: str, task_id: int) -> bool:
        """Giải phóng ô về trạng thái cũ (EMPTY hoặc FULL) khi hủy hoặc gặp lỗi."""
        with self.lock:
            cur = self.conn.cursor()
            cur.execute("BEGIN IMMEDIATE;")
            try:
                cur.execute(
                    """
                    SELECT quantity, item_code, reserved_by_task
                    FROM slots
                    WHERE slot_id = ?;
                    """,
                    (slot_id,),
                )
                row = cur.fetchone()
                if not row:
                    self.conn.rollback()
                    return False

                # Khôi phục trạng thái chuẩn: nếu quantity > 0 và có item_code -> FULL, ngược lại -> EMPTY
                new_state = "FULL" if (row["quantity"] > 0 and row["item_code"]) else "EMPTY"
                cur.execute(
                    """
                    UPDATE slots
                    SET state = ?, reserved_by_task = NULL
                    WHERE slot_id = ?;
                    """,
                    (new_state, slot_id),
                )
                self.conn.commit()
                self.add_event(
                    "INFO",
                    f"Nhiệm vụ {task_id}: Đã giải phóng ô {slot_id} về trạng thái '{new_state}'",
                )
                return True
            except Exception as e:
                self.conn.rollback()
                self.add_event("ERROR", f"Lỗi transaction release_slot: {e}")
                raise

    def commit_inbound(self, task_id: int, slot_id: str, item_code: str) -> bool:
        """Xác nhận nhập kho thành công: slot -> FULL, quantity=1, ghi transactions(+1)."""
        with self.lock:
            cur = self.conn.cursor()
            cur.execute("BEGIN IMMEDIATE;")
            try:
                ts = now_iso()
                cur.execute(
                    """
                    UPDATE slots
                    SET state = 'FULL', quantity = 1, item_code = ?, stored_at = ?, reserved_by_task = NULL
                    WHERE slot_id = ?;
                    """,
                    (item_code, ts, slot_id),
                )
                cur.execute(
                    """
                    INSERT INTO transactions(task_id, type, item_code, slot_id, delta, ts)
                    VALUES (?, 'INBOUND', ?, ?, 1, ?);
                    """,
                    (task_id, item_code, slot_id, ts),
                )
                self.conn.commit()
                self.add_event(
                    "INFO",
                    f"Nhiệm vụ {task_id}: Hoàn tất nhập kho mặt hàng '{item_code}' vào ô {slot_id} (+1)",
                )
                return True
            except Exception as e:
                self.conn.rollback()
                self.add_event("ERROR", f"Lỗi transaction commit_inbound: {e}")
                raise

    def commit_outbound(self, task_id: int, slot_id: str, item_code: str) -> bool:
        """Xác nhận xuất kho thành công: slot -> EMPTY, quantity=0, ghi transactions(-1)."""
        with self.lock:
            cur = self.conn.cursor()
            cur.execute("BEGIN IMMEDIATE;")
            try:
                ts = now_iso()
                cur.execute(
                    """
                    UPDATE slots
                    SET state = 'EMPTY', quantity = 0, item_code = NULL, stored_at = NULL, reserved_by_task = NULL
                    WHERE slot_id = ?;
                    """,
                    (slot_id,),
                )
                cur.execute(
                    """
                    INSERT INTO transactions(task_id, type, item_code, slot_id, delta, ts)
                    VALUES (?, 'OUTBOUND', ?, ?, -1, ?);
                    """,
                    (task_id, item_code, slot_id, ts),
                )
                self.conn.commit()
                self.add_event(
                    "INFO",
                    f"Nhiệm vụ {task_id}: Hoàn tất xuất kho mặt hàng '{item_code}' từ ô {slot_id} (-1)",
                )
                return True
            except Exception as e:
                self.conn.rollback()
                self.add_event("ERROR", f"Lỗi transaction commit_outbound: {e}")
                raise

    # ── Quản lý Nhiệm vụ (Task Queue) ────────────────────────────────────────

    def create_task(self, task_type: str, item_code: str, priority: int = 1) -> int:
        """Tạo nhiệm vụ mới và đưa vào hàng đợi (status='QUEUED')."""
        task_type = task_type.upper()
        ts = now_iso()
        with self.lock:
            with self.conn:
                cur = self.conn.execute(
                    """
                    INSERT INTO tasks(type, item_code, priority, status, created_at)
                    VALUES (?, ?, ?, 'QUEUED', ?);
                    """,
                    (task_type, item_code, priority, ts),
                )
                task_id = cur.lastrowid
            self.add_event("INFO", f"Đã tiếp nhận nhiệm vụ #{task_id}: {task_type} {item_code} (ưu tiên={priority})")
            return task_id

    def get_queued_tasks(self) -> List[Dict[str, Any]]:
        """Lấy danh sách nhiệm vụ đang chờ theo thứ tự ưu tiên (cao -> thấp) và FIFO."""
        cur = self.conn.cursor()
        cur.execute(
            """
            SELECT task_id, type, item_code, slot_id, priority, status, created_at
            FROM tasks
            WHERE status = 'QUEUED'
            ORDER BY priority DESC, task_id ASC;
            """
        )
        return [dict(row) for row in cur.fetchall()]

    def start_task(self, task_id: int, slot_id: Optional[str] = None):
        ts = now_iso()
        with self.lock:
            with self.conn:
                self.conn.execute(
                    """
                    UPDATE tasks
                    SET status = 'RUNNING', started_at = ?, slot_id = coalesce(?, slot_id)
                    WHERE task_id = ?;
                    """,
                    (ts, slot_id, task_id),
                )

    def finish_task(self, task_id: int, status: str = "SUCCEEDED", error: Optional[str] = None):
        ts = now_iso()
        with self.lock:
            with self.conn:
                self.conn.execute(
                    """
                    UPDATE tasks
                    SET status = ?, finished_at = ?, error = ?
                    WHERE task_id = ?;
                    """,
                    (status, ts, error, task_id),
                )
            self.add_event("INFO" if status == "SUCCEEDED" else "WARN", f"Nhiệm vụ #{task_id} kết thúc: {status} ({error or 'OK'})")

    def cancel_task(self, task_id: int) -> bool:
        """Hủy nhiệm vụ nếu chưa chạy xong."""
        with self.lock:
            with self.conn:
                cur = self.conn.execute(
                    "SELECT status, slot_id FROM tasks WHERE task_id = ?;", (task_id,)
                )
                row = cur.fetchone()
                if not row or row["status"] in ("SUCCEEDED", "FAILED", "CANCELLED"):
                    return False
                slot_id = row["slot_id"]

            if slot_id:
                self.release_slot(slot_id, task_id)

            self.finish_task(task_id, status="CANCELLED", error="User cancelled")
            return True

    def get_task(self, task_id: int) -> Optional[Dict[str, Any]]:
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM tasks WHERE task_id = ?;", (task_id,))
        row = cur.fetchone()
        return dict(row) if row else None

    # ── Đọc Tồn kho & Sự kiện ────────────────────────────────────────────────

    def get_inventory(self) -> Dict[str, Any]:
        """Tổng hợp tồn kho và cảnh báo tồn thấp."""
        cur = self.conn.cursor()
        cur.execute(
            """
            SELECT i.item_code, i.name, i.box_tag_id, i.low_stock_threshold,
                   count(case when s.state = 'FULL' then 1 end) as in_stock,
                   count(case when s.state = 'RESERVED' then 1 end) as reserved
            FROM items i
            LEFT JOIN slots s ON i.item_code = s.item_code
            GROUP BY i.item_code;
            """
        )
        items_summary = []
        for r in cur.fetchall():
            d = dict(r)
            d["is_low_stock"] = bool(d["in_stock"] <= d["low_stock_threshold"])
            items_summary.append(d)

        cur_slots = self.conn.cursor()
        cur_slots.execute(
            """
            SELECT count(*) as total,
                   count(case when state = 'FULL' then 1 end) as full_count,
                   count(case when state = 'EMPTY' then 1 end) as empty_count,
                   count(case when state = 'RESERVED' then 1 end) as reserved_count
            FROM slots;
            """
        )
        slot_stats = dict(cur_slots.fetchone())

        return {
            "items": items_summary,
            "slot_stats": slot_stats,
            "timestamp": now_iso(),
        }

    def get_all_slots(self) -> List[Dict[str, Any]]:
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM slots ORDER BY rack, col, tier;")
        return [dict(r) for r in cur.fetchall()]

    def get_slot(self, slot_id: str) -> Optional[Dict[str, Any]]:
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM slots WHERE slot_id = ?;", (slot_id,))
        row = cur.fetchone()
        return dict(row) if row else None

    def get_slot_by_tag(self, slot_tag_id: int) -> Optional[Dict[str, Any]]:
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM slots WHERE slot_tag_id = ?;", (slot_tag_id,))
        row = cur.fetchone()
        return dict(row) if row else None

    def get_item_by_code(self, item_code: str) -> Optional[Dict[str, Any]]:
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM items WHERE item_code = ?;", (item_code,))
        row = cur.fetchone()
        return dict(row) if row else None

    def get_item_by_box_tag(self, box_tag_id: int) -> Optional[Dict[str, Any]]:
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM items WHERE box_tag_id = ?;", (box_tag_id,))
        row = cur.fetchone()
        return dict(row) if row else None

    def add_event(self, level: str, message: str):
        ts = now_iso()
        with self.lock:
            with self.conn:
                self.conn.execute(
                    "INSERT INTO events(level, message, ts) VALUES (?, ?, ?);",
                    (level.upper(), message, ts),
                )

    def get_events(self, limit: int = 50) -> List[Dict[str, Any]]:
        cur = self.conn.cursor()
        cur.execute(
            "SELECT * FROM events ORDER BY id DESC LIMIT ?;", (limit,)
        )
        return [dict(r) for r in cur.fetchall()]

    def get_transactions(self, limit: int = 50) -> List[Dict[str, Any]]:
        cur = self.conn.cursor()
        cur.execute(
            "SELECT * FROM transactions ORDER BY id DESC LIMIT ?;", (limit,)
        )
        return [dict(r) for r in cur.fetchall()]

    def get_all_items(self) -> List[Dict[str, Any]]:
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM items ORDER BY item_code;")
        return [dict(r) for r in cur.fetchall()]

    def get_tasks(self, status: Optional[str] = None, limit: int = 50) -> List[Dict[str, Any]]:
        cur = self.conn.cursor()
        if status:
            cur.execute(
                "SELECT * FROM tasks WHERE status = ? ORDER BY task_id DESC LIMIT ?;",
                (status.upper(), limit),
            )
        else:
            cur.execute(
                "SELECT * FROM tasks ORDER BY task_id DESC LIMIT ?;", (limit,)
            )
        return [dict(r) for r in cur.fetchall()]

    def get_item_stock(self, item_code: str) -> int:
        cur = self.conn.cursor()
        cur.execute(
            "SELECT count(*) FROM slots WHERE state = 'FULL' AND item_code = ?;",
            (item_code,),
        )
        return cur.fetchone()[0]

    def get_free_slots_count(self) -> int:
        cur = self.conn.cursor()
        cur.execute("SELECT count(*) FROM slots WHERE state = 'EMPTY';")
        return cur.fetchone()[0]

    def close(self):
        self.conn.close()
