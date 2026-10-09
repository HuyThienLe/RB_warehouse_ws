#!/usr/bin/env python3
"""gen_waypoints.py – Tự động sinh waypoints.yaml từ slots.yaml và world_params.yaml.

Nội dung sinh ra gồm:
- HOME: (-2.4, 0.0, 0.0)
- IN_STAGING: cách bàn IN 0.5m, hướng về bàn (+y, yaw = pi/2)
- OUT_STAGING: cách bàn OUT 0.5m, hướng về bàn (-y, yaw = -pi/2)
- SLOT_STAGING_<tag_id>: điểm chờ trước từng ô kệ (cách mặt kệ 0.55m, lấy từ slots.yaml)
"""

import os
import sys
import yaml
import math

WS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SLOTS_FILE = os.path.join(WS_DIR, "src", "wh_navigation", "config", "slots.yaml")
WORLD_FILE = os.path.join(WS_DIR, "src", "wh_gazebo", "config", "world_params.yaml")
OUT_FILE = os.path.join(WS_DIR, "src", "wh_navigation", "config", "waypoints.yaml")


def main():
    if not os.path.exists(SLOTS_FILE):
        print(f"[ERROR] Không tìm thấy {SLOTS_FILE}")
        sys.exit(1)

    with open(SLOTS_FILE, "r") as f:
        slots_data = yaml.safe_load(f).get("slots", {})

    with open(WORLD_FILE, "r") as f:
        world_data = yaml.safe_load(f)

    st_in = world_data["stations"]["station_in"]["center"]
    st_out = world_data["stations"]["station_out"]["center"]
    home_pose = world_data["stations"]["home"]["pose"]

    # Bàn IN có tâm (x=-2.4, y=1.2), kích thước 0.5x0.5m -> mép trước bàn tại y = 0.95m
    # Điểm chờ cách bàn 0.5m -> y = 0.95 - 0.50 = 0.45m
    table_in_front_y = float(st_in["y"]) - 0.25
    in_staging = {
        "x": float(st_in["x"]),
        "y": round(table_in_front_y - 0.5, 3),
        "yaw": float(math.pi / 2.0),
    }

    # Bàn OUT có tâm (x=-2.4, y=-1.2), kích thước 0.5x0.5m -> mép trước bàn tại y = -0.95m
    # Điểm chờ cách bàn 0.5m -> y = -0.95 + 0.50 = -0.45m
    table_out_front_y = float(st_out["y"]) + 0.25
    out_staging = {
        "x": float(st_out["x"]),
        "y": round(table_out_front_y + 0.5, 3),
        "yaw": float(-math.pi / 2.0),
    }

    waypoints = {
        "HOME": {
            "x": float(home_pose["x"]),
            "y": float(home_pose["y"]),
            "yaw": float(home_pose["yaw"]),
            "description": "Vị trí chờ ban đầu của robot",
        },
        "IN_STAGING": {
            "x": round(in_staging["x"], 3),
            "y": round(in_staging["y"], 3),
            "yaw": round(in_staging["yaw"], 4),
            "description": "Điểm chờ tiếp nhận hàng trước trạm IN (cách 0.5m)",
        },
        "OUT_STAGING": {
            "x": round(out_staging["x"], 3),
            "y": round(out_staging["y"], 3),
            "yaw": round(out_staging["yaw"], 4),
            "description": "Điểm chờ xuất hàng trước trạm OUT (cách 0.5m)",
        },
    }

    for tag_id, slot in sorted(slots_data.items(), key=lambda x: int(x[0])):
        sp = slot["staging_pose"]
        name = f"SLOT_STAGING_{tag_id}"
        waypoints[name] = {
            "x": float(sp["x"]),
            "y": float(sp["y"]),
            "yaw": float(sp["yaw"]),
            "slot_id": int(tag_id),
            "rack": slot["rack"],
            "col": slot["col"],
            "tier": slot["tier"],
            "description": f"Điểm chờ trước ô kệ {tag_id} ({slot['rack']}, cột {slot['col']}, tầng {slot['tier']})",
        }

    os.makedirs(os.path.dirname(OUT_FILE), exist_ok=True)
    with open(OUT_FILE, "w") as f:
        yaml.dump({"waypoints": waypoints}, f, sort_keys=False, default_flow_style=False)

    print(f"[OK] Đã sinh {len(waypoints)} waypoints vào: {OUT_FILE}")
    for name, wp in waypoints.items():
        print(f"  - {name:18s}: x={wp['x']:6.3f}, y={wp['y']:6.3f}, yaw={wp['yaw']:6.3f}")


if __name__ == "__main__":
    main()
