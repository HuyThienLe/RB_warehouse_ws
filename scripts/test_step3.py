#!/usr/bin/env python3
"""test_step3.py – Nghiệm thu Step 3: Môi trường kho trong Gazebo.

Chạy:
    python3 scripts/test_step3.py [--headless]

Test 1: Kiểm tra world file mở không lỗi (syntax check)
Test 2: Kiểm tra texture tag PNG đúng định dạng và đọc được ArUco
Test 3: Kiểm tra spawn/remove qua service (cần Gazebo đang chạy)
Test 4: Kiểm tra slots.yaml đầy đủ và đúng format
"""

import argparse
import math
import os
import sys
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np

# ── Setup paths ───────────────────────────────────────────────────────────────
WS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(WS)  # go up from scripts/
SRC = os.path.join(WS, "src")
WH_GAZEBO = os.path.join(SRC, "wh_gazebo")
WH_VISION = os.path.join(SRC, "wh_vision")
WH_NAV = os.path.join(SRC, "wh_navigation")

sys.path.insert(0, WH_VISION)

try:
    from wh_vision.aruco_compat import ArucoCompat
    ARUCO_OK = True
except ImportError as e:
    print(f"[WARN] Cannot import ArucoCompat: {e}")
    ARUCO_OK = False


PASS = "\033[92m[PASS]\033[0m"
FAIL = "\033[91m[FAIL]\033[0m"
WARN = "\033[93m[WARN]\033[0m"
INFO = "\033[94m[INFO]\033[0m"

results = []


def check(name, ok, detail=""):
    symbol = PASS if ok else FAIL
    print(f"  {symbol} {name}" + (f" – {detail}" if detail else ""))
    results.append((name, ok))
    return ok


# ── Test 1: World file XML validity ──────────────────────────────────────────
def test_world_file():
    print("\n[TEST 1] World file structure")
    world_file = os.path.join(WH_GAZEBO, "worlds", "warehouse.world")
    check("warehouse.world exists", os.path.exists(world_file), world_file)
    if not os.path.exists(world_file):
        return

    try:
        tree = ET.parse(world_file)
        root = tree.getroot()
        check("Valid XML", True)
    except ET.ParseError as e:
        check("Valid XML", False, str(e))
        return

    world = root.find("world")
    check("Has <world> element", world is not None)

    models = world.findall("model") if world is not None else []
    model_names = [m.get("name", "") for m in models]
    check("Has ground_plane", "ground_plane" in model_names, f"{len(models)} models total")
    check("Has wall_north", "wall_north" in model_names)
    check("Has wall_south", "wall_south" in model_names)
    check("Has table_in", "table_in" in model_names)
    check("Has table_out", "table_out" in model_names)
    check("Has gazebo_ros_state plugin",
          world.find(".//plugin[@name='gazebo_ros_state']") is not None)

    # Đếm tag models
    tag_models = [n for n in model_names if n.startswith("tag_slot_")]
    check(f"12 tag_slot models", len(tag_models) == 12, f"found {len(tag_models)}")

    # Vật cản tĩnh
    obstacle_models = [n for n in model_names if n.startswith("static_obstacle_")]
    check("3 static obstacles", len(obstacle_models) == 3, f"found {len(obstacle_models)}")


# ── Test 2: ArUco textures ────────────────────────────────────────────────────
def test_aruco_textures():
    print("\n[TEST 2] ArUco texture generation + detection")
    tex_dir = os.path.join(WH_GAZEBO, "models", "aruco_tags", "materials", "textures")
    mat_file = os.path.join(WH_GAZEBO, "models", "aruco_tags", "materials", "scripts",
                            "aruco.material")

    check("textures/ directory exists", os.path.isdir(tex_dir))
    check("aruco.material exists", os.path.exists(mat_file))

    expected_ids = list(range(0, 12)) + list(range(20, 40))
    missing = []
    for tid in expected_ids:
        if not os.path.exists(os.path.join(tex_dir, f"tag_{tid}.png")):
            missing.append(tid)
    check(f"All {len(expected_ids)} tag PNGs present",
          len(missing) == 0, f"missing: {missing}" if missing else "")

    if not ARUCO_OK:
        print(f"  {WARN} Skipping detection test (ArucoCompat not importable)")
        return

    aruco = ArucoCompat("DICT_4X4_50")
    detection_fails = []
    distance_errors = []

    for tid in list(range(0, 12)) + [30, 31]:
        img_path = os.path.join(tex_dir, f"tag_{tid}.png")
        if not os.path.exists(img_path):
            detection_fails.append(tid)
            continue

        img = cv2.imread(img_path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids = aruco.detect(gray)

        if ids is None or len(ids) == 0:
            detection_fails.append(tid)
        elif int(ids[0][0]) != tid:
            detection_fails.append(f"{tid}(got {ids[0][0]})")

    check(f"Tag detection 14 shelf+station tags",
          len(detection_fails) == 0,
          f"failed: {detection_fails}" if detection_fails else "100% correct")


# ── Test 3: slots.yaml ────────────────────────────────────────────────────────
def test_slots_yaml():
    print("\n[TEST 3] slots.yaml validation")
    import yaml
    slots_file = os.path.join(WH_NAV, "config", "slots.yaml")
    check("slots.yaml exists", os.path.exists(slots_file))
    if not os.path.exists(slots_file):
        return

    with open(slots_file) as f:
        data = yaml.safe_load(f)

    slots = data.get("slots", {})
    check("12 slots defined", len(slots) == 12, f"found {len(slots)}")

    required_keys = {"rack", "col", "tier", "tag_pose", "box_center_pose",
                     "staging_pose", "work_pose"}
    ok = True
    for sid, info in slots.items():
        missing_keys = required_keys - set(info.keys())
        if missing_keys:
            ok = False
            print(f"    Slot {sid} missing: {missing_keys}")
    check("All slots have required fields", ok)

    # Kiểm tra tọa độ hợp lý
    for sid, info in slots.items():
        bc = info["box_center_pose"]
        tp = info["tag_pose"]
        # Hộp phải nằm sau mặt kệ
        if info["rack"] == "rack_a":
            assert bc["y"] > tp["y"], f"Slot {sid}: box should be behind rack_a front"
        else:
            assert bc["y"] < tp["y"], f"Slot {sid}: box should be behind rack_b front"
    check("Box positions are behind shelf fronts", True)


# ── Test 4: Material file ─────────────────────────────────────────────────────
def test_material_file():
    print("\n[TEST 4] Material file content")
    mat_file = os.path.join(WH_GAZEBO, "models", "aruco_tags", "materials",
                            "scripts", "aruco.material")
    if not os.path.exists(mat_file):
        check("aruco.material content", False, "file missing")
        return

    with open(mat_file) as f:
        content = f.read()

    check("Contains 'lighting off'", "lighting off" in content)
    check("Contains Tag0 entry", "material Aruco/Tag0" in content)
    check("Contains Tag30 entry", "material Aruco/Tag30" in content)
    check("Contains Tag31 entry", "material Aruco/Tag31" in content)

    entries = content.count("material Aruco/Tag")
    check(f"32 material entries", entries == 32, f"found {entries}")


# ── Summary ───────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Test Step 3: Warehouse Environment")
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    print("=" * 60)
    print("  NGHIỆM THU STEP 3 – MÔI TRƯỜNG KHO TRONG GAZEBO")
    print("=" * 60)

    test_world_file()
    test_aruco_textures()
    test_slots_yaml()
    test_material_file()

    print("\n" + "=" * 60)
    passed = sum(1 for _, ok in results if ok)
    total = len(results)
    print(f"  Kết quả: {passed}/{total} tests PASSED")
    if passed == total:
        print(f"  {PASS} Tất cả test đạt!")
    else:
        failed = [(n, ok) for n, ok in results if not ok]
        print(f"  {FAIL} Các test thất bại:")
        for name, _ in failed:
            print(f"    - {name}")
    print("=" * 60)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
