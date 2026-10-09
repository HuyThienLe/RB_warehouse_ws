#!/usr/bin/env python3
"""gen_tags.py – Sinh texture PNG cho ArUco tags và file aruco.material.

Chạy:
    python3 scripts/gen_tags.py

Output:
    wh_gazebo/models/aruco_tags/materials/textures/tag_<id>.png
    wh_gazebo/models/aruco_tags/materials/scripts/aruco.material
"""

import os
import sys

import cv2
import numpy as np

# Thêm wh_vision vào path để dùng ArucoCompat
_ws = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_vision_pkg = os.path.join(_ws, "..", "wh_vision")
sys.path.insert(0, _vision_pkg)

try:
    from wh_vision.aruco_compat import ArucoCompat
except ImportError:
    # Fallback: tự tạo tương thích khi chạy standalone
    class ArucoCompat:
        def __init__(self, dict_name="DICT_4X4_50"):
            a = cv2.aruco
            self.dict = a.getPredefinedDictionary(getattr(a, dict_name))
            self.new_api = hasattr(a, "ArucoDetector")

        def make_marker(self, marker_id, px):
            if hasattr(cv2.aruco, "generateImageMarker"):
                return cv2.aruco.generateImageMarker(self.dict, marker_id, px)
            return cv2.aruco.drawMarker(self.dict, marker_id, px)


# ── Cấu hình ─────────────────────────────────────────────────────────────────
MARKER_PX = 400        # cạnh vùng marker (px)
QUIET_PCT = 0.20       # quiet zone 20% mỗi bên → 0.2*400=80 px để ArUco tách biên chuẩn trên nền kệ
DICT_NAME = "DICT_4X4_50"

# Tag id cần sinh: 0-11 (shelf), 20-39 (box), 30-31 (station, cũng trong 20-39)
TAG_IDS = list(range(0, 12)) + list(range(20, 40))

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PKG_DIR = os.path.dirname(SCRIPT_DIR)
TEX_DIR = os.path.join(PKG_DIR, "models", "aruco_tags", "materials", "textures")
MAT_DIR = os.path.join(PKG_DIR, "models", "aruco_tags", "materials", "scripts")

os.makedirs(TEX_DIR, exist_ok=True)
os.makedirs(MAT_DIR, exist_ok=True)


def make_tag_image(aruco: ArucoCompat, tag_id: int) -> np.ndarray:
    """Tạo ảnh PNG với quiet zone trắng xung quanh marker."""
    marker_img = aruco.make_marker(tag_id, MARKER_PX)
    border = int(MARKER_PX * QUIET_PCT)
    total = MARKER_PX + 2 * border
    canvas = np.ones((total, total), dtype=np.uint8) * 255
    canvas[border:border + MARKER_PX, border:border + MARKER_PX] = marker_img
    # Chuyển sang RGB (3 kênh) để Gazebo texture loader đọc đúng
    canvas_rgb = cv2.cvtColor(canvas, cv2.COLOR_GRAY2RGB)
    return canvas_rgb


def gen_material_script(tag_ids) -> str:
    """Sinh nội dung file aruco.material cho Gazebo/OGRE."""
    lines = []
    for tid in tag_ids:
        lines.append(f"material Aruco/Tag{tid}")
        lines.append("{")
        lines.append("  technique")
        lines.append("  {")
        lines.append("    pass")
        lines.append("    {")
        lines.append("      lighting off")
        lines.append("      texture_unit")
        lines.append("      {")
        lines.append(f"        texture tag_{tid}.png")
        lines.append("      }")
        lines.append("    }")
        lines.append("  }")
        lines.append("}")
        lines.append("")
    return "\n".join(lines)


def main():
    aruco = ArucoCompat(DICT_NAME)
    generated = []

    for tid in TAG_IDS:
        img = make_tag_image(aruco, tid)
        out_path = os.path.join(TEX_DIR, f"tag_{tid}.png")
        cv2.imwrite(out_path, img)
        generated.append(tid)
        print(f"  [OK] tag_{tid}.png  ({img.shape[1]}x{img.shape[0]} px)")

    mat_content = gen_material_script(TAG_IDS)
    mat_path = os.path.join(MAT_DIR, "aruco.material")
    with open(mat_path, "w") as f:
        f.write(mat_content)
    print(f"\n  [OK] aruco.material  ({len(TAG_IDS)} entries)")
    print(f"\nTextures → {TEX_DIR}")
    print(f"Material  → {mat_path}")
    print(f"\nTotal: {len(generated)} tags generated.")


if __name__ == "__main__":
    main()
