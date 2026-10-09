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

from PIL import Image
from pydantic import BaseModel, field_validator
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Query, Response
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
import rclpy
from rclpy.executors import MultiThreadedExecutor

from wh_task_manager.inventory_db import InventoryDB
from wh_task_manager.task_manager_node import find_config_path
from wh_web.web_bridge import WebBridge


# ── Pydantic Request Models ──────────────────────────────────────────────────
class OrderIn(BaseModel):
    item_code: str
    quantity: int = 1

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
    "origin": [-3.5, -2.5, 0.0],
    "width": 280,
    "height": 200,
    "image_url": "/api/map/image",
}
MAP_IMAGE_PATH: Optional[str] = None
cached_map_png: Optional[bytes] = None


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
    """Chuyển đổi file bản đồ PGM sang PNG để trình duyệt hiển thị."""
    global cached_map_png, MAP_IMAGE_PATH
    map_dir = os.path.dirname(__file__)
    png_path = os.path.join(map_dir, "map.png")
    MAP_IMAGE_PATH = png_path

    if os.path.exists(png_path):
        with open(png_path, "rb") as f:
            cached_map_png = f.read()
        return

    # Tìm file warehouse_map.pgm
    pgm_path = find_config_path("wh_navigation/maps/warehouse_map.pgm")
    if pgm_path and os.path.exists(pgm_path):
        try:
            im = Image.open(pgm_path)
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
    global loop, db, ros_thread_handle
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

    # 2. Cache ảnh bản đồ PNG
    init_map_image()

    # 3. Khởi chạy luồng ROS 2
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
    return inv


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

    free_slots = db.get_free_slots_count()
    if free_slots < order.quantity:
        raise HTTPException(
            status_code=409,
            detail=f"Hết ô trống trên kệ (NO_FREE_SLOT): Yêu cầu {order.quantity} ô, hiện chỉ còn {free_slots} ô trống!",
        )

    # Gọi tạo nhiệm vụ qua WebBridge (hoặc DB trực tiếp nếu bridge chưa sẵn sàng)
    if bridge is not None:
        success, task_id, msg = bridge.submit_task("INBOUND", order.item_code, order.quantity)
        if not success:
            raise HTTPException(status_code=400, detail=msg)
    else:
        task_id = db.create_task("INBOUND", order.item_code, quantity=order.quantity)
        msg = f"Đã tiếp nhận nhiệm vụ INBOUND #{task_id}"

    # Broadcast cập nhật ngay
    await broadcast({"type": "inventory", "data": db.get_inventory()})
    await broadcast({
        "type": "notification",
        "data": {
            "level": "info",
            "text": f"Đã tiếp nhận lệnh NHẬP {order.quantity} hộp '{order.item_code}' (Đơn #{task_id})",
        },
    })
    return {"success": True, "task_id": task_id, "message": msg}


@app.post("/api/tasks/outbound")
async def create_outbound_task(order: OrderIn):
    """Tạo nhiệm vụ xuất hàng (Outbound)."""
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
        await broadcast({
            "type": "notification",
            "data": {"level": "warn", "text": f"Đã hủy nhiệm vụ #{task_id}"},
        })
    return {"success": ok, "message": msg}


@app.get("/api/robot")
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


@app.post("/api/estop")
async def trigger_estop():
    """Kích hoạt Dừng khẩn cấp robot."""
    if bridge is not None:
        bridge.trigger_estop()
    return {"status": "ESTOP_TRIGGERED", "message": "Đã kích hoạt dừng khẩn cấp"}


@app.post("/api/estop/reset")
async def reset_estop():
    """Khôi phục sau dừng khẩn cấp."""
    if bridge is not None:
        bridge.reset_estop()
    return {"status": "ESTOP_RESET", "message": "Đã khôi phục trạng thái hoạt động"}


@app.get("/api/map")
async def get_map_metadata():
    """Thông số bản đồ 2D cho Canvas web (resolution, origin, dimensions)."""
    return MAP_METADATA


@app.get("/api/map/image")
async def get_map_image():
    """Trả về file ảnh bản đồ PNG phục vụ vẽ lên canvas."""
    if cached_map_png:
        return Response(content=cached_map_png, media_type="image/png")
    init_map_image()
    if cached_map_png:
        return Response(content=cached_map_png, media_type="image/png")
    raise HTTPException(status_code=404, detail="Không tìm thấy ảnh bản đồ")


# ── Bổ sung Bài 8b: Bản Đồ SLAM Sống & Điều Khiển Dựng Map ─────────────────

@app.get("/api/map/live.png")
async def get_live_map_image():
    """Trả về ảnh PNG phiên bản mới nhất của bản đồ SLAM đang dựng (Bài 8b.3)."""
    if bridge:
        png_data = bridge.get_live_map_png()
        if png_data:
            return Response(content=png_data, media_type="image/png")
    # Fallback bản đồ tĩnh
    if cached_map_png:
        return Response(content=cached_map_png, media_type="image/png")
    init_map_image()
    if cached_map_png:
        return Response(content=cached_map_png, media_type="image/png")
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


@app.post("/api/teleop")
async def teleop_command(cmd: TeleopIn):
    """Điều khiển robot thủ công (Bài 8b.5)."""
    if not bridge:
        raise HTTPException(status_code=503, detail="WebBridge chưa sẵn sàng")
    bridge.send_teleop(cmd.vx, cmd.vy, cmd.wz)
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
static_dir = os.path.join(os.path.dirname(__file__), "static")
if os.path.exists(static_dir):
    app.mount("/", StaticFiles(directory=static_dir, html=True), name="static")


def main():
    """Khởi động Uvicorn Web Server lắng nghe trên 0.0.0.0:8000."""
    import uvicorn
    uvicorn.run("wh_web.app:app", host="0.0.0.0", port=8000, reload=False, log_level="info")


if __name__ == "__main__":
    main()
