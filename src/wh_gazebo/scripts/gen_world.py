#!/usr/bin/env python3
"""gen_world.py – Sinh warehouse.world và slots.yaml từ world_params.yaml.

Chạy từ thư mục gốc workspace:
    python3 src/wh_gazebo/scripts/gen_world.py

Output:
    src/wh_gazebo/worlds/warehouse.world
    src/wh_navigation/config/slots.yaml
"""

import math
import os
import sys
import yaml

# ── Đường dẫn ────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)          # wh_gazebo/
WS_SRC = os.path.dirname(PKG_DIR)             # src/
WS_ROOT = os.path.dirname(WS_SRC)             # warehouse_ws/

PARAMS_FILE = os.path.join(PKG_DIR, "config", "world_params.yaml")
WORLD_OUT = os.path.join(PKG_DIR, "worlds", "warehouse.world")
SLOTS_OUT = os.path.join(WS_SRC, "wh_navigation", "config", "slots.yaml")

# ── Load params ───────────────────────────────────────────────────────────────
with open(PARAMS_FILE) as f:
    P = yaml.safe_load(f)

WH = P["warehouse"]
SHELVES = P["shelves"]
STATIONS = P["stations"]
TAGS = P["tags"]
BOXES_CFG = P["boxes"]

# ── Helper ────────────────────────────────────────────────────────────────────
def _pose_str(x, y, z, roll=0.0, pitch=0.0, yaw=0.0) -> str:
    return f"{x:.4f} {y:.4f} {z:.4f} {roll:.6f} {pitch:.6f} {yaw:.6f}"


# ── 3.3 compute_layout() ─────────────────────────────────────────────────────
def compute_layout() -> dict:
    """Tính toán vị trí ô, tag, hộp, điểm nav cho mọi slot trong kho.

    Returns
    -------
    slots : dict  key=tag_id  value=dict với các trường:
        rack, col, tier, tag_pose, box_center_pose,
        staging_pose, work_pose
    """
    slots = {}
    slot_depth = SHELVES["slot_dimensions"]["depth"]  # 0.22

    for rack_name in ("rack_a", "rack_b"):
        rack = SHELVES[rack_name]
        front_y = rack["front_y"]
        facing = rack["facing_direction"]  # "-y" or "+y"

        for level_name in ("level_1", "level_2"):
            level = rack["levels"][level_name]
            z_face = level["elevation_z"]
            tier = 1 if level_name == "level_1" else 2
            tag_ids_row = level["tag_ids"]

            for col_idx, (col_x, tag_id) in enumerate(
                zip(rack["columns_x"], tag_ids_row)
            ):
                # ── Tag pose ────────────────────────────────────────────
                # Tag đặt ở mặt trước kệ, hướng ra lối đi (aisle)
                # Giá A hướng -y (lối đi y < 0.60) → tag ở y = front_y - 0.001 (nổi về phía -y), yaw = π
                # Giá B hướng +y (lối đi y > -0.60) → tag ở y = front_y + 0.001 (nổi về phía +y), yaw = 0
                if facing == "-y":
                    tag_y = front_y - 0.001
                    tag_yaw = math.pi
                    # hộp nằm về phía y tăng (vào trong lòng kệ)
                    box_y = front_y + slot_depth / 2.0
                    # staging: phía trước kệ A trong hành lang
                    staging_y = front_y - 0.635    # y = 1.125 m
                    staging_yaw = math.pi / 2.0    # quay về hướng +y nhìn vào kệ
                    work_y = front_y - 0.435       # y = 1.325 m (cách tâm hộp 0.545m = 1 cánh tay duỗi thẳng SCARA)
                else:  # "+y"
                    tag_y = front_y + 0.001
                    tag_yaw = 0.0
                    # hộp nằm về phía y giảm (vào trong lòng kệ)
                    box_y = front_y - slot_depth / 2.0
                    # staging: phía trước kệ B trong hành lang
                    staging_y = front_y + 0.635    # y = -1.125 m
                    staging_yaw = -math.pi / 2.0   # quay về hướng -y nhìn vào kệ
                    work_y = front_y + 0.435       # y = -1.325 m (cách tâm hộp 0.545m = 1 cánh tay duỗi thẳng SCARA)

                # Tag đặt ngay dưới mặt sàn đỡ ô (z_face - 0.04)
                tag_z = z_face - 0.04

                # Tâm hộp: z = z_face + half_box (0.04/2 = 0.02) + 0.001 chống rung
                box_z = z_face + BOXES_CFG["dimensions"]["height"] / 2.0 + 0.001

                slots[tag_id] = {
                    "rack": rack_name,
                    "col": col_idx,
                    "tier": tier,
                    "tag_pose": {
                        "x": col_x, "y": tag_y, "z": tag_z,
                        "yaw": tag_yaw,
                    },
                    "box_center_pose": {
                        "x": col_x, "y": box_y, "z": box_z,
                    },
                    "staging_pose": {
                        "x": col_x, "y": staging_y, "yaw": staging_yaw,
                    },
                    "work_pose": {
                        "x": col_x, "y": work_y, "yaw": staging_yaw,
                    },
                }
    # Thêm trạm IN và OUT (Tag 30 và 31) - chuẩn hóa cách 1 sải tay duỗi thẳng SCARA (0.545m từ base)
    slots[30] = {
        "rack": "station_in", "col": 0, "tier": 1,
        "tag_pose": {"x": -2.40, "y": 1.049, "z": 0.150, "yaw": 3.14159},
        "box_center_pose": {"x": -2.40, "y": 1.150, "z": 0.171},
        "staging_pose": {"x": -2.40, "y": 0.450, "yaw": 1.5708},
        "work_pose": {"x": -2.40, "y": 0.605, "yaw": 1.5708},
    }
    slots[31] = {
        "rack": "station_out", "col": 0, "tier": 1,
        "tag_pose": {"x": -2.40, "y": -1.049, "z": 0.150, "yaw": 0.0},
        "box_center_pose": {"x": -2.40, "y": -1.150, "z": 0.171},
        "staging_pose": {"x": -2.40, "y": -0.450, "yaw": -1.5708},
        "work_pose": {"x": -2.40, "y": -0.605, "yaw": -1.5708},
    }
    return slots


# ── SDF builder helpers ───────────────────────────────────────────────────────
def _static_box(name, size_x, size_y, size_z, px, py, pz,
                roll=0.0, pitch=0.0, yaw=0.0,
                color="0.5 0.5 0.5 1.0", cast_shadow="false",
                mu=1.0, has_collision=True) -> str:
    pose = _pose_str(px, py, pz, roll, pitch, yaw)
    col_str = f"""        <collision name="col">
          <geometry><box><size>{size_x} {size_y} {size_z}</size></box></geometry>
          <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>
        </collision>""" if has_collision else ""
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
{col_str}
        <visual name="vis">
          <cast_shadows>{cast_shadow}</cast_shadows>
          <geometry><box><size>{size_x} {size_y} {size_z}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
      </link>
    </model>"""


def _tag_plane(name, tag_id, px, py, pz, yaw=0.0) -> str:
    """Tấm phẳng 0.05×0.001×0.05 với material Aruco/Tag<id>."""
    pose = _pose_str(px, py, pz, 0, 0, yaw)
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <collision name="col">
          <geometry><box><size>0.05 0.001 0.05</size></box></geometry>
        </collision>
        <visual name="vis">
          <cast_shadows>false</cast_shadows>
          <geometry><box><size>0.05 0.001 0.05</size></box></geometry>
          <material>
            <script>
              <uri>model://aruco_tags/materials/scripts</uri>
              <uri>model://aruco_tags/materials/textures</uri>
              <name>Aruco/Tag{tag_id}</name>
            </script>
          </material>
        </visual>
      </link>
    </model>"""


def _build_shelf_tray(name: str, px: float, py: float, pz: float,
                      tray_w: float = 0.36, tray_d: float = 0.18, tray_h: float = 0.024,
                      facing: str = "-y") -> str:
    """Khay đựng hàng đặt trong mỗi ô kệ (Industrial Storage Tray/Tote).
    Đáy phẳng và 3 thành bao (sau, trái, phải). Mặt trước mở để tay kẹp robot gắp hộp.
    pz là cao độ mặt sàn đỡ khay (elevation_z).
    """
    inside_sign = +1.0 if facing == "-y" else -1.0
    wall_t = 0.006
    color = "0.15 0.35 0.65 1.0"      # Industrial Blue
    color_rim = "0.20 0.50 0.85 1.0"  # Lighter blue rim
    back_y_rel = inside_sign * (tray_d / 2.0 - wall_t / 2.0)
    front_y_rel = -inside_sign * (tray_d / 2.0 - wall_t / 2.0)
    side_x_rel = tray_w / 2.0 - wall_t / 2.0
    half_h = tray_h / 2.0

    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{_pose_str(px, py, pz)}</pose>
      <link name="link">
        <!-- Đáy khay mỏng 2mm sát mặt sàn -->
        <visual name="bottom_vis">
          <pose>0 0 0.001 0 0 0</pose>
          <geometry><box><size>{tray_w} {tray_d} 0.002</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
        <!-- Thành sau khay -->
        <visual name="back_wall_vis">
          <pose>0 {back_y_rel} {half_h} 0 0 0</pose>
          <geometry><box><size>{tray_w} {wall_t} {tray_h}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
        <!-- Thành trái khay -->
        <visual name="left_wall_vis">
          <pose>{-side_x_rel} 0 {half_h} 0 0 0</pose>
          <geometry><box><size>{wall_t} {tray_d} {tray_h}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
        <!-- Thành phải khay -->
        <visual name="right_wall_vis">
          <pose>{side_x_rel} 0 {half_h} 0 0 0</pose>
          <geometry><box><size>{wall_t} {tray_d} {tray_h}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>
        </visual>
        <!-- Gờ mép trước thấp 4mm -->
        <visual name="front_lip_vis">
          <pose>0 {front_y_rel} 0.004 0 0 0</pose>
          <geometry><box><size>{tray_w} {wall_t} 0.008</size></box></geometry>
          <material><ambient>{color_rim}</ambient><diffuse>{color_rim}</diffuse></material>
        </visual>
        <!-- Collisions thành khay bảo vệ hộp -->
        <collision name="back_col">
          <pose>0 {back_y_rel} {half_h} 0 0 0</pose>
          <geometry><box><size>{tray_w} {wall_t} {tray_h}</size></box></geometry>
        </collision>
        <collision name="left_col">
          <pose>{-side_x_rel} 0 {half_h} 0 0 0</pose>
          <geometry><box><size>{wall_t} {tray_d} {tray_h}</size></box></geometry>
        </collision>
        <collision name="right_col">
          <pose>{side_x_rel} 0 {half_h} 0 0 0</pose>
          <geometry><box><size>{wall_t} {tray_d} {tray_h}</size></box></geometry>
        </collision>
      </link>
    </model>"""


def _floor_marker(name, px, py, color_rgba, size=0.3) -> str:
    """Vạch màu sàn (visual only) cho IN/OUT/HOME."""
    r, g, b, a = color_rgba
    pose = _pose_str(px, py, 0.001)
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <visual name="vis">
          <cast_shadows>false</cast_shadows>
          <geometry><box><size>{size} {size} 0.001</size></box></geometry>
          <material>
            <ambient>{r} {g} {b} {a}</ambient>
            <diffuse>{r} {g} {b} {a}</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def _docking_bay_box(name, px, py, length_x=0.36, width_y=0.48, color_rgba=(0.95, 0.75, 0.10, 0.9)) -> str:
    """Ô vạch line dừng xe trước ô kệ (Industrial AGV Docking Bay Line Marker).
    Bao gồm 4 viền màu vàng phản quang đánh dấu chuẩn vị trí dừng robot.
    """
    r, g, b, a = color_rgba
    border_t = 0.02
    half_lx = length_x / 2.0
    half_wy = width_y / 2.0
    pose = _pose_str(px, py, 0.001)

    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <!-- Viền trước -->
        <visual name="border_front">
          <pose>0 {half_wy - border_t/2.0} 0 0 0 0</pose>
          <cast_shadows>false</cast_shadows>
          <geometry><box><size>{length_x} {border_t} 0.001</size></box></geometry>
          <material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse></material>
        </visual>
        <!-- Viền sau -->
        <visual name="border_back">
          <pose>0 {-half_wy + border_t/2.0} 0 0 0 0</pose>
          <cast_shadows>false</cast_shadows>
          <geometry><box><size>{length_x} {border_t} 0.001</size></box></geometry>
          <material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse></material>
        </visual>
        <!-- Viền trái -->
        <visual name="border_left">
          <pose>{-half_lx + border_t/2.0} 0 0 0 0 0</pose>
          <cast_shadows>false</cast_shadows>
          <geometry><box><size>{border_t} {width_y} 0.001</size></box></geometry>
          <material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse></material>
        </visual>
        <!-- Viền phải -->
        <visual name="border_right">
          <pose>{half_lx - border_t/2.0} 0 0 0 0 0</pose>
          <cast_shadows>false</cast_shadows>
          <geometry><box><size>{border_t} {width_y} 0.001</size></box></geometry>
          <material><ambient>{r} {g} {b} {a}</ambient><diffuse>{r} {g} {b} {a}</diffuse></material>
        </visual>
        <!-- Vạch tâm căn chỉnh (Alignment Cross Line) -->
        <visual name="center_mark">
          <pose>0 0 0 0 0 0</pose>
          <cast_shadows>false</cast_shadows>
          <geometry><box><size>0.08 0.08 0.001</size></box></geometry>
          <material><ambient>{r} {g} {b} 0.5</ambient><diffuse>{r} {g} {b} 0.5</diffuse></material>
        </visual>
      </link>
    </model>"""


# ── Shelf geometry ────────────────────────────────────────────────────────────
def _build_shelf(rack_name: str, rack: dict, slots: dict) -> str:
    """Sinh SDF cho một giá kệ hoàn chỉnh bao gồm vách sau, các tầng đỡ, vách hông và vách chia ô."""
    sdf = []
    front_y = rack["front_y"]
    facing = rack["facing_direction"]
    cols_x = rack["columns_x"]  # [0.75, 1.25, 1.75]

    # Các cột có tâm tại 0.75, 1.25, 1.75 (bước cột 0.50 m)
    # Ô 0: [0.50, 1.00] (tâm 0.75)
    # Ô 1: [1.00, 1.50] (tâm 1.25)
    # Ô 2: [1.50, 2.00] (tâm 1.75)
    min_x = 0.50
    max_x = 2.00
    shelf_w = max_x - min_x      # 1.50 m
    cx = (min_x + max_x) / 2.0   # 1.25 m
    shelf_d = 0.22               # chiều sâu ô kệ
    back_thickness = 0.02
    side_thickness = 0.02
    plank_thickness = 0.02

    # Hướng vào trong kệ
    if facing == "-y":
        inside_sign = +1.0   # kệ A: bên trong ở y lớn hơn
    else:
        inside_sign = -1.0   # kệ B: bên trong ở y nhỏ hơn

    back_y = front_y + inside_sign * (shelf_d + back_thickness / 2.0)
    shelf_plank_y = front_y + inside_sign * (shelf_d / 2.0)

    # 1. Tường sau kệ (che toàn bộ chiều rộng kệ)
    sdf.append(_static_box(
        f"{rack_name}_back_wall",
        shelf_w + 2 * side_thickness, back_thickness, 0.60,
        cx, back_y, 0.30,
        color="0.6 0.45 0.30 1.0", cast_shadow="false", mu=0.5
    ))

    # 2. Tấm sàn các tầng đỡ hàng (Level 1 và Level 2)
    for level_name in ("level_1", "level_2"):
        lev = rack["levels"][level_name]
        z = lev["elevation_z"]
        sdf.append(_static_box(
            f"{rack_name}_{level_name}_plank",
            shelf_w, shelf_d, plank_thickness,
            cx, shelf_plank_y, z - plank_thickness / 2.0,
            color="0.55 0.40 0.25 1.0", cast_shadow="false", mu=0.7
        ))

    # 3. Tấm sàn đáy
    sdf.append(_static_box(
        f"{rack_name}_floor_plank",
        shelf_w, shelf_d, plank_thickness,
        cx, shelf_plank_y, 0.005,
        color="0.55 0.40 0.25 1.0", cast_shadow="false", mu=0.7
    ))

    # 4. Tấm vách biên trái / phải
    left_x = min_x - side_thickness / 2.0
    right_x = max_x + side_thickness / 2.0
    side_y = shelf_plank_y

    for side_name, sx in (("left", left_x), ("right", right_x)):
        sdf.append(_static_box(
            f"{rack_name}_{side_name}_side",
            side_thickness, shelf_d, 0.60,
            sx, side_y, 0.30,
            color="0.6 0.45 0.30 1.0", cast_shadow="false", mu=0.5
        ))

    # 5. Vách ngăn chia cột ô (ngăn cách cột 0-1 tại x=1.00 và cột 1-2 tại x=1.50)
    divider_thickness = 0.015
    for div_idx, div_x in enumerate([1.00, 1.50]):
        sdf.append(_static_box(
            f"{rack_name}_divider_{div_idx}",
            divider_thickness, shelf_d, 0.60,
            div_x, side_y, 0.30,
            color="0.6 0.45 0.30 1.0", cast_shadow="false", mu=0.5
        ))

    # 6. Thanh xà viền trước kệ đỡ tag (mặt trước ngay dưới mỗi tầng)
    for level_name in ("level_1", "level_2"):
        lev = rack["levels"][level_name]
        z = lev["elevation_z"]
        sdf.append(_static_box(
            f"{rack_name}_{level_name}_front_beam",
            shelf_w, 0.01, 0.04,
            cx, front_y + inside_sign * 0.005, z - 0.04,
            color="0.50 0.38 0.24 1.0", cast_shadow="false", mu=0.5,
            has_collision=False
        ))

    # 7. Tags ô ArUco
    for level_name in ("level_1", "level_2"):
        lev = rack["levels"][level_name]
        for col_x, tag_id in zip(cols_x, lev["tag_ids"]):
            slot = slots[tag_id]
            tp = slot["tag_pose"]
            sdf.append(_tag_plane(
                f"tag_slot_{tag_id}",
                tag_id,
                tp["x"], tp["y"], tp["z"],
                yaw=tp["yaw"]
            ))

    # 8. Khay đựng hàng chuyên dụng cho từng ô kệ (Industrial Storage Trays)
    for level_name in ("level_1", "level_2"):
        lev = rack["levels"][level_name]
        z = lev["elevation_z"]
        for col_x, tag_id in zip(cols_x, lev["tag_ids"]):
            sdf.append(_build_shelf_tray(
                f"shelf_tray_{tag_id}",
                col_x, shelf_plank_y, z,
                tray_w=0.36, tray_d=0.18, tray_h=0.024,
                facing=facing
            ))

    return "\n".join(sdf)


# ── Station (IN/OUT) ──────────────────────────────────────────────────────────
def _build_station(name: str, cfg: dict) -> str:
    cx, cy = cfg["center"]["x"], cfg["center"]["y"]
    tag_id = cfg["tag_id"]
    sdf = []

    # Bàn công nghiệp rộng rãi (0.85m x 0.40m, cao 0.15m) chứa thoải mái 6 phôi/hộp chống rơi 100%
    table_w = 0.85
    table_d = 0.40
    table_h = 0.15
    sdf.append(_static_box(
        f"table_{name}",
        table_w, table_d, table_h,
        cx, cy, table_h / 2.0,
        color="0.80 0.80 0.85 1.0", cast_shadow="false", mu=0.9
    ))

    # Khay tiếp nhận hàng cỡ lớn trên mặt bàn trạm IN/OUT (0.76m x 0.32m cho 6 phôi)
    facing_dir = "-y" if cy > 0 else "+y"
    sdf.append(_build_shelf_tray(
        f"station_tray_{name}",
        cx, cy, table_h,
        tray_w=0.76, tray_d=0.32, tray_h=0.025,
        facing=facing_dir
    ))

    # Tag dán thẳng đứng ở mép bàn hướng về phía robot (lối đi y = 0)
    # Bàn IN (cy > 0): mép trước ở cy - table_d/2 -> tag ở mép trước
    # Bàn OUT (cy < 0): mép trước ở cy + table_d/2 -> tag ở mép trước
    if cy > 0:
        tag_y = cy - table_d / 2.0 - 0.001   # mép phía y nhỏ hơn
        tag_yaw = math.pi    # quay về -y
    else:
        tag_y = cy + table_d / 2.0 + 0.001   # mép phía y lớn hơn
        tag_yaw = 0.0        # quay về +y

    sdf.append(_tag_plane(
        f"tag_{name}",
        tag_id,
        cx, tag_y, table_h,
        yaw=tag_yaw
    ))

    return "\n".join(sdf)


# ── World template ────────────────────────────────────────────────────────────
def build_world(slots: dict) -> str:
    wh_dim = WH["dimensions"]
    lx, ly = wh_dim["length_x"], wh_dim["width_y"]
    wall_h = WH["walls"]["height"]
    wall_t = WH["walls"]["thickness"]

    parts = []

    # ── Tường bao ─────────────────────────────────────────────────────────────
    # Bắc (y+)
    parts.append(_static_box("wall_north", lx + wall_t*2, wall_t, wall_h,
                              0, ly/2 + wall_t/2, wall_h/2,
                              color="0.75 0.75 0.80 1.0", cast_shadow="false"))
    # Nam (y-)
    parts.append(_static_box("wall_south", lx + wall_t*2, wall_t, wall_h,
                              0, -ly/2 - wall_t/2, wall_h/2,
                              color="0.75 0.75 0.80 1.0", cast_shadow="false"))
    # Đông (x+)
    parts.append(_static_box("wall_east", wall_t, ly, wall_h,
                              lx/2 + wall_t/2, 0, wall_h/2,
                              color="0.75 0.75 0.80 1.0", cast_shadow="false"))
    # Tây (x-)
    parts.append(_static_box("wall_west", wall_t, ly, wall_h,
                              -lx/2 - wall_t/2, 0, wall_h/2,
                              color="0.75 0.75 0.80 1.0", cast_shadow="false"))

    # ── Kệ ────────────────────────────────────────────────────────────────────
    parts.append(_build_shelf("rack_a", SHELVES["rack_a"], slots))
    parts.append(_build_shelf("rack_b", SHELVES["rack_b"], slots))

    # ── Trạm IN / OUT ─────────────────────────────────────────────────────────
    parts.append(_build_station("in", STATIONS["station_in"]))
    parts.append(_build_station("out", STATIONS["station_out"]))

    # ── Vạch sàn IN / OUT / HOME ──────────────────────────────────────────────
    si = STATIONS["station_in"]["center"]
    so = STATIONS["station_out"]["center"]
    sh = STATIONS["home"]["pose"]
    parts.append(_floor_marker("floor_in",   si["x"], si["y"], (0.0, 1.0, 0.0, 0.7)))
    parts.append(_floor_marker("floor_out",  so["x"], so["y"], (1.0, 0.5, 0.0, 0.7)))
    parts.append(_floor_marker("floor_home", sh["x"], sh["y"], (0.0, 0.5, 1.0, 0.7)))

    # ── Vạch ô line đỗ xe chuẩn trước kệ hàng (Docking Bay Line Markers) ─────
    for col_idx, cx in enumerate(SHELVES["rack_a"]["columns_x"]):
        parts.append(_docking_bay_box(f"docking_bay_rack_a_col_{col_idx}", cx, 1.325, length_x=0.36, width_y=0.48))
    for col_idx, cx in enumerate(SHELVES["rack_b"]["columns_x"]):
        parts.append(_docking_bay_box(f"docking_bay_rack_b_col_{col_idx}", cx, -1.325, length_x=0.36, width_y=0.48))


    # ── Vật cản tĩnh: để trống, vật cản được sinh động qua /wh/spawn_obstacle ───
    obstacles = []
    for i, (ox, oy, oz) in enumerate(obstacles):
        parts.append(_static_box(
            f"static_obstacle_{i+1}",
            0.30, 0.30, 0.30,
            ox, oy, oz,
            color="0.80 0.20 0.20 1.0", cast_shadow="false", mu=0.8
        ))

    models_sdf = "\n".join(parts)

    world_sdf = f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="warehouse">

    <!-- ── Vật lý ──────────────────────────────────────────────────────── -->
    <physics type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_update_rate>500</real_time_update_rate>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
        </constraints>
      </ode>
    </physics>

    <!-- ── Plugin trạng thái ───────────────────────────────────────────── -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>20</update_rate>
    </plugin>

    <!-- ── Ánh sáng ────────────────────────────────────────────────────── -->
    <light name="sun" type="directional">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.85 0.85 0.85 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <light name="point_nw" type="point">
      <cast_shadows>false</cast_shadows>
      <pose>-1.5 1.5 3.0 0 0 0</pose>
      <diffuse>0.6 0.6 0.65 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <attenuation><range>8</range><constant>0.3</constant><linear>0.05</linear><quadratic>0.0</quadratic></attenuation>
    </light>
    <light name="point_ne" type="point">
      <cast_shadows>false</cast_shadows>
      <pose>1.5 1.5 3.0 0 0 0</pose>
      <diffuse>0.6 0.6 0.65 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <attenuation><range>8</range><constant>0.3</constant><linear>0.05</linear><quadratic>0.0</quadratic></attenuation>
    </light>
    <light name="point_sw" type="point">
      <cast_shadows>false</cast_shadows>
      <pose>-1.5 -1.5 3.0 0 0 0</pose>
      <diffuse>0.6 0.6 0.65 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <attenuation><range>8</range><constant>0.3</constant><linear>0.05</linear><quadratic>0.0</quadratic></attenuation>
    </light>
    <light name="point_se" type="point">
      <cast_shadows>false</cast_shadows>
      <pose>1.5 -1.5 3.0 0 0 0</pose>
      <diffuse>0.6 0.6 0.65 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <attenuation><range>8</range><constant>0.3</constant><linear>0.05</linear><quadratic>0.0</quadratic></attenuation>
    </light>

    <!-- ── Sàn ─────────────────────────────────────────────────────────── -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="col">
          <geometry><plane><normal>0 0 1</normal><size>8.0 6.0</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="vis">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>8.0 6.0</size></plane></geometry>
          <material>
            <ambient>0.85 0.85 0.85 1</ambient>
            <diffuse>0.85 0.85 0.85 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- ── Models ──────────────────────────────────────────────────────── -->
{models_sdf}

  </world>
</sdf>
"""
    return world_sdf


# ── slots.yaml ────────────────────────────────────────────────────────────────
def build_slots_yaml(slots: dict) -> str:
    """Sinh nội dung slots.yaml cho wh_navigation."""
    data = {"slots": {}}
    for tag_id, info in sorted(slots.items()):
        data["slots"][tag_id] = {
            "rack": info["rack"],
            "col": info["col"],
            "tier": info["tier"],
            "tag_pose": info["tag_pose"],
            "box_center_pose": info["box_center_pose"],
            "staging_pose": {
                k: round(v, 4) for k, v in info["staging_pose"].items()
            },
            "work_pose": {
                k: round(v, 4) for k, v in info["work_pose"].items()
            },
        }
    return yaml.dump(data, default_flow_style=False, sort_keys=False)


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    print("=== gen_world.py ===")
    print(f"  Params   : {PARAMS_FILE}")

    slots = compute_layout()
    print(f"  Slots    : {len(slots)} ô được tính ({sorted(slots.keys())})")

    world_content = build_world(slots)
    os.makedirs(os.path.dirname(WORLD_OUT), exist_ok=True)
    with open(WORLD_OUT, "w") as f:
        f.write(world_content)
    print(f"  World    : {WORLD_OUT}")

    slots_content = build_slots_yaml(slots)
    os.makedirs(os.path.dirname(SLOTS_OUT), exist_ok=True)
    with open(SLOTS_OUT, "w") as f:
        f.write(slots_content)
    print(f"  Slots    : {SLOTS_OUT}")
    print("Done.")


if __name__ == "__main__":
    main()
