#!/usr/bin/env python3
"""app.py – FastAPI Web Server quản lý kho và điều phối robot (Bài 8.1 - 8.3 & 8b).

Cung cấp:
- REST API (JSON, Pydantic validation): Quản lý tồn kho, mặt hàng, nhiệm vụ, E-Stop, bản đồ.
- Bổ sung Bài 8b: Chế độ vận hành (navigation|mapping), bản đồ SLAM sống, điểm LiDAR, teleop, lưu map.
- WebSocket /ws: Phát sóng trạng thái thời gian thực (robot pose 5-10 Hz, scan 5 Hz, map_update, trail, path, inventory).
- Tích hợp rclpy (WebBridge) trong background thread với MultiThreadedExecutor.
- Phục vụ Static files (HTML, CSS, JS) cho thiết bị di động trong cùng mạng LAN.
"""

import os
import io
import json
import socket
import asyncio
import threading
import subprocess
from typing import Optional, Set, List, Dict, Any
import yaml

# Đảm bảo cấu hình DDS và ROS_DOMAIN_ID đồng bộ tuyệt đối với full_system
os.environ.setdefault("ROS_DOMAIN_ID", "42")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")

from PIL import Image
from pydantic import BaseModel, field_validator
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Query, Response
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, StreamingResponse
import rclpy
from rclpy.executors import MultiThreadedExecutor

from wh_task_manager.inventory_db import InventoryDB
from wh_task_manager.task_manager_node import find_config_path
from wh_web.web_bridge import WebBridge, make_placeholder_frame


# ── Pydantic Request Models ──────────────────────────────────────────────────
class OrderIn(BaseModel):
    item_code: str
    quantity: int = 1
    slot_id: Optional[str] = None

    @field_validator("quantity")
    @classmethod
    def validate_quantity(cls, v: int) -> int:
        if v < 1 or v > 10:
            raise ValueError("Số lượng (quantity) phải nằm trong khoảng từ 1 đến 10.")
        return v


class TeleopIn(BaseModel):
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


class RobotEnableIn(BaseModel):
    enabled: bool


class SlamSaveIn(BaseModel):
    name: str = "warehouse_map"


class SlamSerializeIn(BaseModel):
    name: str = "slam_pose_graph"


# ── Khởi tạo FastAPI App ─────────────────────────────────────────────────────
app = FastAPI(
    title="Warehouse Management System",
    description="Hệ thống Điều phối Nhiệm vụ và Quản lý Tồn kho Tự động",
    version="1.0.0",
)

# Cho phép CORS để dễ dàng debug
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Biến toàn cục
ws_clients: Set[WebSocket] = set()
loop: Optional[asyncio.AbstractEventLoop] = None
bridge: Optional[WebBridge] = None
db: Optional[InventoryDB] = None
ros_thread_handle: Optional[threading.Thread] = None
auto_mapper_proc: Optional[subprocess.Popen] = None

# Thông số bản đồ tĩnh (warehouse_map)
MAP_METADATA: Dict[str, Any] = {
    "resolution": 0.025,
    "origin": [-4.3, -2.5, 0.0],
    "width": 312,
    "height": 200,
    "image_url": "/api/map/image",
}
MAP_IMAGE_PATH: Optional[str] = None
cached_map_png: Optional[bytes] = None
MAP_NO_CACHE_HEADERS = {
    "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
    "Pragma": "no-cache",
    "Expires": "0",
}


def get_local_ip() -> str:
    """Lấy IP mạng LAN thực tế của máy tính."""
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def init_map_image():
    """Chuyển đổi file bản đồ PGM sang PNG để trình duyệt hiển thị.

    PNG chỉ là cache: luôn dựng lại khi PGM nguồn mới hơn (hoặc PNG chưa có) để
    bản đồ web không bao giờ lệch so với bản đồ Nav2 / Gazebo hiện tại.
    """
    global cached_map_png, MAP_IMAGE_PATH, MAP_METADATA
    map_dir = os.path.dirname(__file__)
    png_path = os.path.join(map_dir, "map.png")
    MAP_IMAGE_PATH = png_path

    # Đọc thông số bản đồ từ warehouse_map.yaml nếu có
    yaml_path = find_config_path("wh_navigation/maps/warehouse_map.yaml")
    if yaml_path and os.path.exists(yaml_path):
        try:
            with open(yaml_path, "r", encoding="utf-8") as yf:
                ydata = yaml.safe_load(yf)
            if ydata:
                MAP_METADATA["resolution"] = float(ydata.get("resolution", 0.025))
                orig = ydata.get("origin", [-4.3, -2.5, 0.0])
                MAP_METADATA["origin"] = [float(orig[0]), float(orig[1]), float(orig[2])]
        except Exception as e:
            print(f"Lỗi đọc warehouse_map.yaml: {e}")

    # Tìm file warehouse_map.pgm
    pgm_path = find_config_path("wh_navigation/maps/warehouse_map.pgm")
    pgm_ok = bool(pgm_path and os.path.exists(pgm_path))

    if pgm_ok:
        try:
            im = Image.open(pgm_path)
            MAP_METADATA["width"] = im.width
            MAP_METADATA["height"] = im.height
        except Exception:
            pass

    if os.path.exists(png_path):
        png_fresh = (not pgm_ok) or (os.path.getmtime(png_path) >= os.path.getmtime(os.path.realpath(pgm_path)))
        if png_fresh:
            with open(png_path, "rb") as f:
                cached_map_png = f.read()
            return

    if pgm_ok:
        try:
            im = Image.open(pgm_path)
            MAP_METADATA["width"] = im.width
            MAP_METADATA["height"] = im.height
            buf = io.BytesIO()
            im.save(buf, format="PNG")
            cached_map_png = buf.getvalue()
            with open(png_path, "wb") as f:
                f.write(cached_map_png)
        except Exception as e:
            print(f"Lỗi chuyển đổi PGM -> PNG: {e}")


async def broadcast(message: dict):
    """Gửi thông điệp tới tất cả các client WebSocket đang kết nối."""
    if not ws_clients:
        return
    text = json.dumps(message)
    dead_clients = []
    for client in list(ws_clients):
        try:
            await client.send_text(text)
        except Exception:
            dead_clients.append(client)
    for dc in dead_clients:
        ws_clients.discard(dc)


def run_ros_bridge():
    """Hàm chạy trong luồng ROS 2 độc lập."""
    global bridge
    try:
        if not rclpy.ok():
            rclpy.init()

        def push_handler(msg_dict: dict):
            if loop and loop.is_running():
                loop.call_soon_threadsafe(asyncio.create_task, broadcast(msg_dict))

        bridge = WebBridge(db=db, push_callback=push_handler)
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(bridge)
        executor.spin()
    except Exception as e:
        print(f"Lỗi trong ros_thread: {e}")
    finally:
        if bridge:
            bridge.destroy_node()


# ── Vòng đời FastAPI (Startup / Shutdown) ───────────────────────────────────
@app.on_event("startup")
async def on_startup():
    global loop, db, ros_thread_handle, bridge
    loop = asyncio.get_running_loop()

    # 1. Khởi tạo CSDL SQLite chung với task_manager (nếu chưa được cấu hình trước bởi test)
    if db is None:
        db_dir = os.path.expanduser("~/.warehouse")
        os.makedirs(db_dir, exist_ok=True)
        db_path = os.path.join(db_dir, "inventory.db")
        db = InventoryDB(db_path)
        db._owned_by_app = True
        schema_file = find_config_path("wh_task_manager/config/schema.sql")
        db.init_schema(schema_file)
        if not db.get_all_slots():
            slots_file = find_config_path("wh_navigation/config/slots.yaml")
            items_file = find_config_path("wh_task_manager/config/items_seed.yaml")
            seed_inv_file = find_config_path("wh_gazebo/config/inventory_seed.yaml")
            db.load_seed_data(slots_file, items_file, seed_inv_file)

        # Dọn dẹp các task bị treo từ phiên chạy cũ và reset các ô RESERVED treo
        with db.lock:
            with db.conn:
                db.conn.execute("UPDATE tasks SET status = 'CANCELLED' WHERE status IN ('QUEUED', 'RUNNING');")
                db.conn.execute("UPDATE slots SET state = 'EMPTY', item_code = NULL, quantity = 0, reserved_by_task = NULL WHERE state = 'RESERVED';")

    # 2. Cache ảnh bản đồ PNG
    init_map_image()

    # 3. Khởi chạy luồng ROS 2 (chỉ chạy duy nhất 1 luồng)
    if ros_thread_handle is None or not ros_thread_handle.is_alive():
        bridge = None
        ros_thread_handle = threading.Thread(target=run_ros_bridge, daemon=True)
        ros_thread_handle.start()

    # Chờ bridge khởi tạo xong (tối đa 2.0s)
    for _ in range(40):
        if bridge is not None:
            break
        await asyncio.sleep(0.05)

    lan_ip = get_local_ip()
    print("=" * 65)
    print("  🚀 HỆ THỐNG QUẢN LÝ KHO (WAREHOUSE WEB PORTAL) ĐÃ KHỞI ĐỘNG")
    print(f"  🌐 Truy cập cục bộ: http://127.0.0.1:8000")
    print(f"  📱 Truy cập từ điện thoại (Cùng Wi-Fi): http://{lan_ip}:8000")
    print("=" * 65)


@app.on_event("shutdown")
async def on_shutdown():
    global db, auto_mapper_proc
    if auto_mapper_proc and auto_mapper_proc.poll() is None:
        try:
            auto_mapper_proc.terminate()
        except Exception:
            pass
        auto_mapper_proc = None

    if db and getattr(db, "_owned_by_app", False):
        try:
            db.close()
        except Exception:
            pass
        db = None


# ── REST API Endpoints (Mục 8.2 & 8b.3, 8b.5) ───────────────────────────────

@app.get("/api/mode")
async def get_system_mode():
    """Lấy chế độ vận hành hiện tại (navigation | mapping) (Bài 8b.1)."""
    current = bridge.current_mode if bridge else "navigation"
    return {"mode": current}


@app.get("/api/inventory")
async def get_inventory():
    """Tồn kho theo ô + tổng theo mã hàng + cờ sắp hết."""
    inv = db.get_inventory()
    inv["slots_detail"] = db.get_all_slots()
    inv["station_info"] = {
        "inbound_capacity": 6,
        "outbound_capacity": 6,
    }
    return inv


@app.post("/api/station/outbound/clear")
async def api_clear_station_outbound():
    """Dọn sạch bàn trả hàng trạm OUT."""
    if not bridge:
        raise HTTPException(status_code=503, detail="WebBridge chưa sẵn sàng")
    if not bridge.is_robot_enabled:
        raise HTTPException(status_code=403, detail="Hệ thống đang ở trạng thái KHÓA (Enable OFF). Vui lòng bật Enable!")
    ok, msg = bridge.clear_station_out()
    return {"success": ok, "message": msg}


@app.post("/api/station/inbound/replenish")
async def api_replenish_station_inbound():
    """Tiếp thêm phôi vào trạm IN."""
    if not bridge:
        raise HTTPException(status_code=503, detail="WebBridge chưa sẵn sàng")
    if not bridge.is_robot_enabled:
        raise HTTPException(status_code=403, detail="Hệ thống đang ở trạng thái KHÓA (Enable OFF). Vui lòng bật Enable!")
    ok, msg = bridge.replenish_station_in()
    return {"success": ok, "message": msg}



@app.get("/api/items")
async def get_items():
    """Danh mục mặt hàng và ngưỡng cảnh báo."""
    items = db.get_all_items()
    for item in items:
        item["current_stock"] = db.get_item_stock(item["item_code"])
        item["is_low_stock"] = bool(item["current_stock"] <= item["low_stock_threshold"])
    return items


@app.post("/api/tasks/inbound")
async def create_inbound_task(order: OrderIn):
    """Tạo nhiệm vụ nhập hàng (Inbound)."""
    if bridge and not bridge.is_robot_enabled:
        raise HTTPException(
            status_code=403,
            detail="Hệ thống đang ở trạng thái KHÓA (Enable OFF). Vui lòng bật Enable để điều phối!",
        )

    if bridge and bridge.current_mode == "mapping":
        raise HTTPException(
            status_code=400,
            detail="Hệ thống đang ở chế độ Dựng bản đồ (mapping). Không tiếp nhận nhiệm vụ kho!",
        )

    item_info = db.get_item_by_code(order.item_code)
    if not item_info:
        raise HTTPException(
            status_code=404,
            detail=f"Mã hàng '{order.item_code}' không tồn tại trong danh mục!",
        )

    # Quy tắc: Hàng nào có hàng rồi thì không được nhập nữa, chưa có hàng mới được nhập
    current_stock = db.get_item_stock(order.item_code)
    if current_stock > 0:
        raise HTTPException(
            status_code=409,
            detail=f"Mặt hàng '{order.item_code}' đã có {current_stock} hộp trong kho (ALREADY_IN_STOCK)! Hệ thống chỉ cho phép nhập mặt hàng chưa có hàng (tồn kho = 0).",
        )

    free_slots = db.get_free_slots_count()
    if free_slots < order.quantity:
        raise HTTPException(
            status_code=409,
            detail=f"Hết ô trống trên kệ (NO_FREE_SLOT): Yêu cầu {order.quantity} ô, hiện chỉ còn {free_slots} ô trống!",
        )

    if order.slot_id:
        target_slot = db.get_slot(order.slot_id)
        if not target_slot:
            raise HTTPException(status_code=404, detail=f"Ô kệ '{order.slot_id}' không tồn tại!")
        if target_slot["state"] != "EMPTY":
            raise HTTPException(
                status_code=409,
                detail=f"Ô kệ '{order.slot_id}' hiện không trống (Trạng thái: {target_slot['state']})!",
            )

    # Gọi tạo nhiệm vụ qua WebBridge (hoặc DB trực tiếp nếu bridge chưa sẵn sàng)
    if bridge is not None:
        success, task_id, msg = bridge.submit_task("INBOUND", order.item_code, order.quantity, slot_id=order.slot_id)
        if not success:
            raise HTTPException(status_code=400, detail=msg)
    else:
        task_id = db.create_task("INBOUND", order.item_code, slot_id=order.slot_id)
        msg = f"Đã tiếp nhận nhiệm vụ INBOUND #{task_id}"

    # Broadcast cập nhật ngay
    await broadcast({"type": "inventory", "data": db.get_inventory()})
    await broadcast({"type": "task", "data": {"task_id": task_id, "action": "created"}})
    slot_notice = f" vào ô {order.slot_id}" if order.slot_id else ""
    await broadcast({
        "type": "notification",
        "data": {
            "level": "info",
            "text": f"Đã tiếp nhận lệnh NHẬP {order.quantity} hộp '{order.item_code}'{slot_notice} (Đơn #{task_id})",
        },
    })
    return {"success": True, "task_id": task_id, "message": msg}


@app.post("/api/tasks/outbound")
async def create_outbound_task(order: OrderIn):
    """Tạo nhiệm vụ xuất hàng (Outbound)."""
    if bridge and not bridge.is_robot_enabled:
        raise HTTPException(
            status_code=403,
            detail="Hệ thống đang ở trạng thái KHÓA (Enable OFF). Vui lòng bật Enable để điều phối!",
        )

    if bridge and bridge.current_mode == "mapping":
        raise HTTPException(
            status_code=400,
            detail="Hệ thống đang ở chế độ Dựng bản đồ (mapping). Không tiếp nhận nhiệm vụ kho!",
        )

    item_info = db.get_item_by_code(order.item_code)
    if not item_info:
        raise HTTPException(
            status_code=404,
            detail=f"Mã hàng '{order.item_code}' không tồn tại trong danh mục!",
        )

    current_stock = db.get_item_stock(order.item_code)
    if current_stock < order.quantity:
        raise HTTPException(
            status_code=409,
            detail=f"Hết hàng mã '{order.item_code}' (OUT_OF_STOCK): Yêu cầu xuất {order.quantity} hộp, hiện chỉ có {current_stock} hộp trong kho!",
        )

    # Gọi tạo nhiệm vụ qua WebBridge (hoặc DB trực tiếp nếu bridge chưa sẵn sàng)
    if bridge is not None:
        success, task_id, msg = bridge.submit_task("OUTBOUND", order.item_code, order.quantity)
        if not success:
            raise HTTPException(status_code=400, detail=msg)
    else:
        task_id = db.create_task("OUTBOUND", order.item_code, quantity=order.quantity)
        msg = f"Đã tiếp nhận nhiệm vụ OUTBOUND #{task_id}"

    # Broadcast cập nhật ngay
    await broadcast({"type": "inventory", "data": db.get_inventory()})
    await broadcast({"type": "task", "data": {"task_id": task_id, "action": "created"}})
    await broadcast({
        "type": "notification",
        "data": {
            "level": "info",
            "text": f"Đã tiếp nhận lệnh XUẤT {order.quantity} hộp '{order.item_code}' (Đơn #{task_id})",
        },
    })
    return {"success": True, "task_id": task_id, "message": msg}


@app.get("/api/tasks")
async def list_tasks(status: Optional[str] = Query(None), limit: int = 50):
    """Danh sách nhiệm vụ theo trạng thái và thời gian."""
    return db.get_tasks(status=status, limit=limit)


@app.post("/api/tasks/{task_id}/cancel")
async def cancel_task(task_id: int):
    """Hủy nhiệm vụ đang chạy hoặc đang trong hàng đợi."""
    task = db.get_task(task_id)
    if not task:
        raise HTTPException(status_code=404, detail=f"Không tìm thấy nhiệm vụ #{task_id}")

    if bridge is not None:
        ok, msg = bridge.cancel_task(task_id)
    else:
        ok = db.cancel_task(task_id)
        msg = f"Đã hủy nhiệm vụ #{task_id}" if ok else f"Không thể hủy nhiệm vụ #{task_id}"

    if ok:
        await broadcast({"type": "inventory", "data": db.get_inventory()})
        await broadcast({"type": "task", "data": {"task_id": task_id, "action": "cancelled"}})
        await broadcast({
            "type": "notification",
            "data": {"level": "warn", "text": f"Đã hủy nhiệm vụ #{task_id}"},
        })
    return {"success": ok, "message": msg}


@app.get("/api/robot")
@app.get("/api/robot/status")
async def get_robot():
    """Trạng thái robot hiện tại (pose, state, task_id, battery)."""
    if bridge is not None:
        return bridge.get_robot_status()
    return {
        "x": 0.0,
        "y": 0.0,
        "yaw": 0.0,
        "state": "IDLE",
        "task_id": 0,
        "holding_box": False,
        "battery_pct": 100,
        "is_estopped": False,
    }


@app.get("/api/transactions")
async def get_transactions(limit: int = 50):
    """Lịch sử biến động kho và giao dịch."""
    return db.get_transactions(limit=limit)


@app.get("/api/events")
async def get_events(limit: int = 50):
    """Lịch sử thông báo và sự kiện kho thời gian thực."""
    return db.get_events(limit=limit)


@app.post("/api/estop")
async def trigger_estop():
    """Kích hoạt Dừng khẩn cấp robot."""
    if bridge is not None:
        bridge.trigger_estop()
    return {"status": "ESTOP_TRIGGERED", "message": "Đã kích hoạt dừng khẩn cấp"}


@app.post("/api/estop/reset")
async def reset_estop():
    """Khôi phục sau dừng khẩn cấp: giải phóng E-Stop và tự động điều hướng robot về trạm HOME rồi mới hoạt động tiếp."""
    if bridge is not None:
        bridge.reset_estop()
    return {"status": "ESTOP_RESET", "message": "Đã giải phóng E-Stop. Robot đang tự động di chuyển về trạm HOME để sẵn sàng hoạt động tiếp."}


@app.get("/api/map")
async def get_map_metadata():
    """Thông số bản đồ 2D cho Canvas web (resolution, origin, dimensions)."""
    return MAP_METADATA


@app.get("/api/map/image")
async def get_map_image():
    """Trả về file ảnh bản đồ PNG phục vụ vẽ lên canvas."""
    init_map_image()  # tự dựng lại nếu PGM nguồn mới hơn cache
    if cached_map_png:
        return Response(content=cached_map_png, media_type="image/png", headers=MAP_NO_CACHE_HEADERS)
    raise HTTPException(status_code=404, detail="Không tìm thấy ảnh bản đồ")


# ── Bổ sung Bài 8b: Bản Đồ SLAM Sống & Điều Khiển Dựng Map ─────────────────

@app.get("/api/map/live.png")
async def get_live_map_image():
    """Trả về ảnh PNG phiên bản mới nhất của bản đồ SLAM đang dựng (Bài 8b.3)."""
    if bridge:
        png_data = bridge.get_live_map_png()
        if png_data:
            return Response(content=png_data, media_type="image/png", headers=MAP_NO_CACHE_HEADERS)
    # Fallback bản đồ tĩnh
    init_map_image()
    if cached_map_png:
        return Response(content=cached_map_png, media_type="image/png", headers=MAP_NO_CACHE_HEADERS)
    raise HTTPException(status_code=404, detail="Chưa có dữ liệu bản đồ trực tiếp")


@app.get("/api/map/live/meta")
async def get_live_map_metadata():
    """Metadata phiên bản bản đồ trực tiếp kèm version (Bài 8b.3)."""
    if bridge:
        meta = bridge.get_live_map_meta()
        if meta.get("version", 0) > 0:
            m = dict(meta)
            m["image_url"] = f"/api/map/live.png?v={m['version']}"
            return m
    # Fallback metadata bản đồ tĩnh
    return {
        "version": 0,
        "width": MAP_METADATA["width"],
        "height": MAP_METADATA["height"],
        "resolution": MAP_METADATA["resolution"],
        "origin_x": MAP_METADATA["origin"][0],
        "origin_y": MAP_METADATA["origin"][1],
        "image_url": "/api/map/image",
    }


def reset_warehouse_history_and_inventory():
    """Reset toàn bộ lịch sử kho hàng (tasks, transactions, events) về 0 theo yêu cầu người dùng."""
    global db, bridge
    if db is None:
        return
    try:
        with db.lock:
            with db.conn:
                db.conn.execute("DELETE FROM transactions;")
                db.conn.execute("DELETE FROM tasks;")
                db.conn.execute("DELETE FROM events;")
                try:
                    db.conn.execute("DELETE FROM sqlite_sequence WHERE name IN ('tasks', 'transactions', 'events');")
                except Exception:
                    pass
                db.conn.execute("UPDATE slots SET state = 'EMPTY', quantity = 0, item_code = NULL, stored_at = NULL, reserved_by_task = NULL WHERE state = 'RESERVED';")
                
                # Nạp lại seed data
                slots_file = find_config_path("wh_navigation/config/slots.yaml")
                items_file = find_config_path("wh_task_manager/config/items_seed.yaml")
                seed_inv_file = find_config_path("wh_gazebo/config/inventory_seed.yaml")
                db.load_seed_data(slots_file, items_file, seed_inv_file)
    except Exception as e:
        print(f"[Reset] Lỗi khi reset lịch sử kho hàng: {e}")

    if bridge:
        bridge.active_task_id = 0
        with bridge.lock:
            bridge.latest_status["task_id"] = 0
            bridge.latest_status["slot_id"] = ""
            bridge.latest_status["holding_box"] = False
            bridge.latest_status["held_box_tag"] = 0
            bridge.latest_status["last_error"] = ""
            bridge.trail = []
            bridge.path_points = []


@app.post("/api/robot/enable")
async def robot_enable_endpoint(data: RobotEnableIn):
    """Bật/tắt công tắc kích hoạt di chuyển của robot (Enable switch)."""
    if not bridge:
        raise HTTPException(status_code=503, detail="WebBridge chưa sẵn sàng")
    
    # RÀNG BUỘC AN TOÀN: Không được phép tắt Enable khi robot đang chạy hoặc đang về HOME
    if not data.enabled and bridge.is_robot_enabled:
        is_running, reason = bridge.is_robot_running()
        if is_running:
            raise HTTPException(
                status_code=400,
                detail=f"Không được phép tắt ENABLE khi robot đang hoạt động! ({reason}). Vui lòng chờ robot dừng hẳn hoặc về trạm HOME an toàn."
            )

    was_enabled = bridge.is_robot_enabled
    try:
        enabled = bridge.set_robot_enabled(data.enabled)
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))

    # Nếu bật lên từ OFF sang ON: reset toàn bộ lịch sử kho hàng về 0 (Bài 8b/Yêu cầu)
    if enabled and not was_enabled:
        reset_warehouse_history_and_inventory()
        await broadcast({"type": "history_reset", "data": {}})
        await broadcast({"type": "inventory", "data": db.get_inventory()})
        await broadcast({"type": "task", "data": {"action": "reset"}})
        await broadcast({
            "type": "notification",
            "data": {
                "level": "success",
                "text": "⚡ ĐÃ BẬT ENABLE: Mở khóa toàn diện hệ thống & reset lịch sử kho hàng về 0!",
            },
        })

    await broadcast({
        "type": "robot_enable",
        "data": {"enabled": enabled}
    })
    return {"success": True, "is_enabled": enabled}


@app.post("/api/teleop")
async def teleop_command(cmd: TeleopIn):
    """Điều khiển robot thủ công (Bài 8b.5)."""
    if not bridge:
        raise HTTPException(status_code=503, detail="WebBridge chưa sẵn sàng")
    if not bridge.is_robot_enabled and (abs(cmd.vx) > 0.001 or abs(cmd.vy) > 0.001 or abs(cmd.wz) > 0.001):
        raise HTTPException(status_code=403, detail="Robot chưa được bật công tắc Enable. Vui lòng bật Enable để di chuyển!")
    ok = bridge.send_teleop(cmd.vx, cmd.vy, cmd.wz)
    if not ok and (abs(cmd.vx) > 0.001 or abs(cmd.vy) > 0.001 or abs(cmd.wz) > 0.001):
        raise HTTPException(status_code=403, detail="Robot đang dừng khẩn cấp (E-STOP) hoặc chưa bật công tắc Enable!")
    return {"success": True, "vx": cmd.vx, "vy": cmd.vy, "wz": cmd.wz}


@app.post("/api/slam/save")
async def slam_save_map(data: SlamSaveIn):
    """Lưu bản đồ SLAM Toolbox ra file .pgm và .yaml (Bài 8b.5)."""
    if not bridge:
        raise HTTPException(status_code=503, detail="WebBridge chưa sẵn sàng")
    ok, msg, path = bridge.save_slam_map(data.name)
    if ok:
        await broadcast({"type": "notification", "data": {"level": "info", "text": msg}})
        return {"success": True, "message": msg, "path": path}
    raise HTTPException(status_code=500, detail=msg)


@app.post("/api/slam/serialize")
async def slam_serialize_map(data: SlamSerializeIn):
    """Lưu pose graph SLAM Toolbox (Bài 8b.5)."""
    if not bridge:
        raise HTTPException(status_code=503, detail="WebBridge chưa sẵn sàng")
    ok, msg = bridge.serialize_slam_map(data.name)
    if ok:
        return {"success": True, "message": msg}
    raise HTTPException(status_code=500, detail=msg)


@app.post("/api/slam/clear")
async def slam_clear_session():
    """Xóa phiên SLAM hiện tại (Bài 8b.5)."""
    if not bridge:
        raise HTTPException(status_code=503, detail="WebBridge chưa sẵn sàng")
    ok, msg = bridge.clear_slam()
    if ok:
        await broadcast({"type": "notification", "data": {"level": "warn", "text": msg}})
        return {"success": True, "message": msg}
    raise HTTPException(status_code=500, detail=msg)


@app.post("/api/mapping/auto/start")
async def start_auto_mapping():
    """Bắt đầu chạy kịch bản AutoMapper quét kho tự động (Bài 8b.5)."""
    global auto_mapper_proc
    if auto_mapper_proc is not None and auto_mapper_proc.poll() is None:
        return {"success": True, "message": "AutoMapper đang hoạt động"}

    script_path = "/home/lehuythien/warehouse_ws/scripts/auto_mapper.py"
    if not os.path.exists(script_path):
        raise HTTPException(status_code=404, detail="Không tìm thấy scripts/auto_mapper.py")

    try:
        auto_mapper_proc = subprocess.Popen(["python3", script_path])
        await broadcast({
            "type": "notification",
            "data": {"level": "info", "text": "Đã bắt đầu Tự động quét kho (AutoMapper)!"},
        })
        return {"success": True, "message": "Đã khởi động AutoMapper"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Lỗi khởi động AutoMapper: {e}")


@app.post("/api/mapping/auto/stop")
async def stop_auto_mapping():
    """Dừng kịch bản AutoMapper (Bài 8b.5)."""
    global auto_mapper_proc
    if auto_mapper_proc is not None and auto_mapper_proc.poll() is None:
        auto_mapper_proc.terminate()
        auto_mapper_proc = None
        if bridge:
            bridge.send_teleop(0.0, 0.0, 0.0)
        await broadcast({
            "type": "notification",
            "data": {"level": "info", "text": "Đã dừng Tự động quét kho (AutoMapper)"},
        })
        return {"success": True, "message": "Đã dừng AutoMapper"}
    return {"success": True, "message": "AutoMapper hiện không chạy"}


# ── Vision & Arm Endpoints (Mục 8.4 & Vision Live Stream) ─────────────────────
class ArmGripperIn(BaseModel):
    state: bool = True


class ArmPoseIn(BaseModel):
    pose: str = "STOW"


async def mjpeg_frame_generator(is_dock: bool):
    """Generator phát luồng video MJPEG liên tục cho trình duyệt."""
    while True:
        if bridge:
            frame = bridge.get_dock_frame() if is_dock else bridge.get_hand_frame()
        else:
            frame = make_placeholder_frame("Đang kết nối ROS 2...")
        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
        )
        await asyncio.sleep(0.08)  # ~12 FPS mượt mà và nhẹ tải mạng


@app.get("/api/vision/dock/stream")
async def stream_dock_camera():
    """Luồng video trực tiếp từ Dock Camera (Camera sườn xe căn chỉnh ArUco)."""
    return StreamingResponse(
        mjpeg_frame_generator(is_dock=True),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/api/vision/hand/stream")
async def stream_hand_camera():
    """Luồng video trực tiếp từ Hand Camera (Camera bàn tay SCARA Eye-in-Hand)."""
    return StreamingResponse(
        mjpeg_frame_generator(is_dock=False),
        media_type="multipart/x-mixed-replace; boundary=frame",
    )


@app.get("/api/vision/dock/live.jpg")
async def get_dock_frame_snapshot():
    """Ảnh tĩnh tức thời từ Dock Camera."""
    frame = bridge.get_dock_frame() if bridge else make_placeholder_frame("Chưa kết nối ROS")
    return Response(content=frame, media_type="image/jpeg", headers={"Cache-Control": "no-cache"})


@app.get("/api/vision/hand/live.jpg")
async def get_hand_frame_snapshot():
    """Ảnh tĩnh tức thời từ Hand Camera."""
    frame = bridge.get_hand_frame() if bridge else make_placeholder_frame("Chưa kết nối ROS")
    return Response(content=frame, media_type="image/jpeg", headers={"Cache-Control": "no-cache"})


@app.get("/api/vision/status")
async def get_vision_telemetry():
    """Dữ liệu nhận diện ArUco tags từ cả 2 camera."""
    if not bridge:
        return {"dock_tags": [], "hand_tags": [], "has_dock_feed": False, "has_hand_feed": False}
    return bridge.get_vision_telemetry()


@app.post("/api/arm/gripper")
async def control_arm_gripper(req: ArmGripperIn):
    """Điều khiển bật / tắt giác hút chân không từ web."""
    if not bridge:
        raise HTTPException(status_code=503, detail="WebBridge chưa sẵn sàng")
    ok, msg = bridge.trigger_gripper(req.state)
    await broadcast({
        "type": "notification",
        "data": {
            "level": "info" if ok else "warn",
            "text": f"Tay kẹp: {msg}",
        },
    })
    return {"success": ok, "message": msg}


@app.post("/api/arm/pose")
async def control_arm_pose(req: ArmPoseIn):
    """Điều khiển tay máy SCARA về tư thế cấu hình trước."""
    if not bridge:
        raise HTTPException(status_code=503, detail="WebBridge chưa sẵn sàng")
    ok, msg = bridge.trigger_arm_pose(req.pose)
    if not ok:
        raise HTTPException(status_code=400, detail=msg)
    await broadcast({
        "type": "notification",
        "data": {
            "level": "info",
            "text": f"Tay máy: {msg}",
        },
    })
    return {"success": ok, "message": msg}


# ── WebSocket Endpoint (Mục 8.3 & 8b.3) ─────────────────────────────────────
@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    ws_clients.add(ws)

    # Gửi ngay gói dữ liệu ban đầu khi client vừa kết nối
    try:
        if bridge:
            await ws.send_text(json.dumps({"type": "mode", "data": {"mode": bridge.current_mode}}))
            await ws.send_text(json.dumps({"type": "robot", "data": bridge.get_robot_status()}))
            meta = bridge.get_live_map_meta()
            if meta.get("version", 0) > 0:
                await ws.send_text(json.dumps({"type": "map_update", "data": meta}))
            if bridge.trail:
                await ws.send_text(json.dumps({"type": "trail", "data": {"points": bridge.trail}}))
        else:
            await ws.send_text(json.dumps({"type": "mode", "data": {"mode": "navigation"}}))

        if db:
            await ws.send_text(json.dumps({"type": "inventory", "data": db.get_inventory()}))
    except Exception:
        pass

    try:
        while True:
            # Lắng nghe tin nhắn từ client (ping / heartbeat)
            data = await ws.receive_text()
            if data == "ping":
                await ws.send_text(json.dumps({"type": "pong"}))
    except WebSocketDisconnect:
        ws_clients.discard(ws)
    except Exception:
        ws_clients.discard(ws)


# ── Phục vụ Giao diện Tĩnh (Static Files - Mục 8.4) ──────────────────────────
src_static_candidate = "/home/lehuythien/RB_warehouse_ws/src/wh_web/wh_web/static"
if os.path.exists(src_static_candidate):
    static_dir = src_static_candidate
else:
    static_dir = os.path.join(os.path.dirname(__file__), "static")
if os.path.exists(static_dir):
    NO_CACHE_HEADERS = {
        "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
        "Pragma": "no-cache",
        "Expires": "0",
    }

    @app.get("/", include_in_schema=False)
    async def serve_root():
        return FileResponse(os.path.join(static_dir, "index.html"), headers=NO_CACHE_HEADERS)

    @app.get("/app.js", include_in_schema=False)
    async def serve_app_js():
        return FileResponse(
            os.path.join(static_dir, "app.js"),
            media_type="application/javascript",
            headers=NO_CACHE_HEADERS,
        )

    @app.get("/style.css", include_in_schema=False)
    async def serve_style_css():
        return FileResponse(
            os.path.join(static_dir, "style.css"),
            media_type="text/css",
            headers=NO_CACHE_HEADERS,
        )

    app.mount("/", StaticFiles(directory=static_dir, html=True), name="static")


def main():
    """Khởi động Uvicorn Web Server lắng nghe trên 0.0.0.0:8000."""
    import uvicorn
    uvicorn.run("wh_web.app:app", host="0.0.0.0", port=8000, reload=False, log_level="info")


if __name__ == "__main__":
    main()
