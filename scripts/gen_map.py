#!/usr/bin/env python3
"""gen_map.py – Rasterize world_params.yaml thành bản đồ 2D (PGM + YAML) cho Nav2.

Độ phân giải: 0.025 m/pixel.
Mặt trước 2 giá kệ: y = +0.60 và y = -0.60 (sai số 0 mm).
Lối đi giữa 2 giá kệ: 1.20 m (y in [-0.60, +0.60]).
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

    # Giới hạn bản đồ có biên an toàn xung quanh
    x_min, x_max = -3.5, 3.5  # 7.0m
    y_min, y_max = -2.5, 2.5  # 5.0m

    width = int(round((x_max - x_min) / res))   # 280 pixels
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

    # 1. Không gian mở trong lòng kho (-3.0 <= x <= 3.0, -2.0 <= y <= 2.0)
    fill_box(-3.0, 3.0, -2.0, 2.0, 254)

    # 2. Tường bao (dày 0.1m)
    fill_box(-3.1, 3.1, 2.0, 2.1, 0)    # Bắc
    fill_box(-3.1, 3.1, -2.1, -2.0, 0)  # Nam
    fill_box(3.0, 3.1, -2.0, 2.0, 0)    # Đông
    fill_box(-3.1, -3.0, -2.0, 2.0, 0)  # Tây

    # 3. Giá kệ A (x in [0.49, 2.01], y in [1.76, 2.00]) -> áp sát tường Bắc (mặt trước tại y = +1.76)
    fill_box(0.49, 2.01, 1.76, 2.00, 0)

    # 4. Giá kệ B (x in [0.49, 2.01], y in [-2.00, -1.76]) -> áp sát tường Nam (mặt trước tại y = -1.76)
    fill_box(0.49, 2.01, -2.00, -1.76, 0)

    # 5. Bàn trạm IN (-2.4, 1.25), kích thước 0.70x0.40 (x in [-2.75, -2.05], y in [1.05, 1.45])
    fill_box(-2.75, -2.05, 1.05, 1.45, 0)

    # 6. Bàn trạm OUT (-2.4, -1.25), kích thước 0.70x0.40 (x in [-2.75, -2.05], y in [-1.45, -1.05])
    fill_box(-2.75, -2.05, -1.45, -1.05, 0)

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
        "origin": [x_min, y_min, 0.0],
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
    print(f"  - Mặt trước Kệ A: y = +1.760m | Mặt trước Kệ B: y = -1.760m (Lối đi: 3.520m)")


if __name__ == "__main__":
    main()
