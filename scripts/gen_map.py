#!/usr/bin/env python3
"""gen_map.py – Rasterize world_params.yaml thành bản đồ 2D (PGM + YAML) cho Nav2.

Độ phân giải: 0.025 m/pixel.
Mặt trước 2 giá kệ: y = +1.76 và y = -1.76 (áp sát tường Bắc/Nam).
Lối đi giữa 2 giá kệ: 3.52 m (y in [-1.76, +1.76]).
"""

import os
import sys
import numpy as np
import yaml

WS_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAP_DIR = os.path.join(WS_DIR, "src", "wh_navigation", "maps")
WORLD_FILE = os.path.join(WS_DIR, "src", "wh_gazebo", "config", "world_params.yaml")


def main():
    os.makedirs(MAP_DIR, exist_ok=True)
    res = 0.025  # 25 mm/pixel

    with open(WORLD_FILE) as f:
        P = yaml.safe_load(f)

    wh_dim = P["warehouse"]["dimensions"]
    x_min_wh = float(wh_dim.get("x_min", -wh_dim["length_x"] / 2.0))
    x_max_wh = float(wh_dim.get("x_max", wh_dim["length_x"] / 2.0))
    y_min_wh = float(wh_dim.get("y_min", -wh_dim["width_y"] / 2.0))
    y_max_wh = float(wh_dim.get("y_max", wh_dim["width_y"] / 2.0))
    wall_t = float(P["warehouse"]["walls"]["thickness"])

    # Giới hạn bản đồ có biên an toàn 0.5m ngoài tường kho
    margin = 0.5
    x_min = x_min_wh - margin  # -3.8 - 0.5 = -4.3m
    x_max = x_max_wh + margin  #  3.0 + 0.5 =  3.5m
    y_min = y_min_wh - margin  # -2.0 - 0.5 = -2.5m
    y_max = y_max_wh + margin  #  2.0 + 0.5 =  2.5m

    width = int(round((x_max - x_min) / res))   # 312 pixels
    height = int(round((y_max - y_min) / res))  # 200 pixels

    # Giá trị quy chuẩn ROS:
    # 205 = Unknown (ngoài tường kho)
    # 254 = Free (trong kho)
    # 0   = Occupied (tường, kệ, bàn, vật cản)
    grid = np.full((height, width), 205, dtype=np.uint8)

    def to_pixel(x, y):
        col = int(round((x - x_min) / res))
        row = (height - 1) - int(round((y - y_min) / res))
        return col, row

    def fill_box(x1, x2, y1, y2, val):
        c1, r2 = to_pixel(x1, y1)
        c2, r1 = to_pixel(x2, y2)
        r_start = max(0, min(r1, r2))
        r_end = min(height, max(r1, r2) + 1)
        c_start = max(0, min(c1, c2))
        c_end = min(width, max(c1, c2) + 1)
        grid[r_start:r_end, c_start:c_end] = val

    # 1. Không gian mở trong lòng kho (-3.8 <= x <= 3.0, -2.0 <= y <= 2.0)
    fill_box(x_min_wh, x_max_wh, y_min_wh, y_max_wh, 254)

    # 2. Tường bao (dày 0.1m)
    fill_box(x_min_wh - wall_t, x_max_wh + wall_t, y_max_wh, y_max_wh + wall_t, 0)    # Bắc
    fill_box(x_min_wh - wall_t, x_max_wh + wall_t, y_min_wh - wall_t, y_min_wh, 0)  # Nam
    fill_box(x_max_wh, x_max_wh + wall_t, y_min_wh, y_max_wh, 0)                    # Đông
    fill_box(x_min_wh - wall_t, x_min_wh, y_min_wh, y_max_wh, 0)                    # Tây

    # 3. Giá kệ A (x in [0.49, 2.01], y in [1.76, 2.00]) -> áp sát tường Bắc (mặt trước tại y = +1.76)
    fill_box(0.49, 2.01, 1.76, 2.00, 0)

    # 4. Giá kệ B (x in [0.49, 2.01], y in [-2.00, -1.76]) -> áp sát tường Nam (mặt trước tại y = -1.76)
    fill_box(0.49, 2.01, -2.00, -1.76, 0)

    # 5. Bàn trạm IN (-2.4, 1.25), kích thước 0.85x0.40 (x in [-2.825, -1.975], y in [1.05, 1.45])
    fill_box(-2.825, -1.975, 1.05, 1.45, 0)

    # 6. Bàn trạm OUT (-2.4, -1.25), kích thước 0.85x0.40 (x in [-2.825, -1.975], y in [-1.45, -1.05])
    fill_box(-2.825, -1.975, -1.45, -1.05, 0)

    # Không gian di chuyển tự do hoàn toàn trong kho (không có vật cản tĩnh hardcode trên map)

    # Lưu file PGM (P5 binary)
    pgm_path = os.path.join(MAP_DIR, "warehouse_map.pgm")
    with open(pgm_path, "wb") as f:
        header = f"P5\n{width} {height}\n255\n"
        f.write(header.encode("ascii"))
        f.write(grid.tobytes())

    # Lưu file YAML
    yaml_path = os.path.join(MAP_DIR, "warehouse_map.yaml")
    map_meta = {
        "image": "warehouse_map.pgm",
        "resolution": res,
        "origin": [round(x_min, 4), round(y_min, 4), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    with open(yaml_path, "w") as f:
        yaml.dump(map_meta, f, sort_keys=False)

    print(f"[OK] Đã sinh bản đồ thành công:")
    print(f"  - PGM : {pgm_path} ({width}x{height} pixels)")
    print(f"  - YAML: {yaml_path}")
    print(f"  - Resolution: {res} m/pixel | Origin: [{x_min}, {y_min}, 0.0]")
    print(f"  - Khoảng mở: x in [{x_min_wh}, {x_max_wh}], y in [{y_min_wh}, {y_max_wh}]")
    print(f"  - Mặt trước Kệ A: y = +1.760m | Mặt trước Kệ B: y = -1.760m (Lối đi: 3.520m)")


if __name__ == "__main__":
    main()
