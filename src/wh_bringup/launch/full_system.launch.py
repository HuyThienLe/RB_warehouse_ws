#!/usr/bin/env python3
"""full_system.launch.py – Khởi chạy toàn bộ hệ thống Kho thông minh AMR (Bài 09.1).

Thứ tự khởi động (đồng bộ & có kiểm soát thời gian):
  1. Gazebo + world + hộp theo seed file.
  2. Robot spawn + joint_state_broadcaster + arm_controller + gripper + slip + EKF.
  3. Nav2 (map_server, AMCL, planner, controller, collision monitor, twist_mux) + đặt pose HOME.
  4. Vision (2 detectors: dock + hand) + dock_server.
  5. pick_place_server (điều khiển tay máy SCARA TRR).
  6. task_manager (CSDL SQLite + FSM điều phối).
  7. Web Portal (FastAPI + WebSocket + giao diện di động).
  8. (Tùy chọn) RViz2 với rviz/full_system.rviz.
  9. In thông báo: HỆ THỐNG SẴN SÀNG — mở http://<IP>:8000.
"""

import os
import socket
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


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


def print_ready_banner(context, *args, **kwargs):
    lan_ip = get_local_ip()
    print("\n" + "=" * 68)
    print("  🚀 HỆ THỐNG SẴN SÀNG — mở giao diện Web tại:")
    print(f"     🌐 Trình duyệt máy tính:  http://127.0.0.1:8000")
    print(f"     📱 Thiết bị di động (Wi-Fi): http://{lan_ip}:8000")
    print("=" * 68 + "\n")
    return []


def generate_launch_description():
    pkg_wh_bringup = get_package_share_directory("wh_bringup")
    pkg_wh_gazebo = get_package_share_directory("wh_gazebo")
    pkg_wh_nav = get_package_share_directory("wh_navigation")
    pkg_wh_vision = get_package_share_directory("wh_vision")
    pkg_wh_manip = get_package_share_directory("wh_manipulation")
    pkg_wh_task = get_package_share_directory("wh_task_manager")
    pkg_wh_web = get_package_share_directory("wh_web")

    default_world = os.path.join(pkg_wh_gazebo, "worlds", "warehouse.world")
    default_seed = os.path.join(pkg_wh_gazebo, "config", "inventory_seed.yaml")
    rviz_config_file = os.path.join(pkg_wh_bringup, "rviz", "full_system.rviz")

    # ── Launch Arguments ─────────────────────────────────────────────────────
    mode = LaunchConfiguration("mode")
    headless = LaunchConfiguration("headless")
    rviz = LaunchConfiguration("rviz")
    enable_slip = LaunchConfiguration("enable_slip")
    seed_file = LaunchConfiguration("seed_file")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_mode = DeclareLaunchArgument(
        "mode", default_value="navigation", description="Operational mode: navigation | mapping"
    )
    declare_headless = DeclareLaunchArgument(
        "headless", default_value="false", description="Run Gazebo headless without GUI"
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch RViz2 with full_system.rviz"
    )
    declare_enable_slip = DeclareLaunchArgument(
        "enable_slip", default_value="true", description="Enable mecanum wheel slip simulation"
    )
    declare_seed_file = DeclareLaunchArgument(
        "seed_file", default_value=default_seed, description="Path to seed boxes YAML"
    )
    declare_world = DeclareLaunchArgument(
        "world", default_value=default_world, description="Path to Gazebo .world file"
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock"
    )

    is_nav_mode = IfCondition(PythonExpression(["'", mode, "' in ('navigation', 'nav')"]))

    # 1. Khởi chạy Mô phỏng Gazebo (World + AMR Robot + Controllers + Slip + EKF + BoxManager)
    warehouse_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_wh_gazebo, "launch", "warehouse.launch.py")
        ),
        launch_arguments={
            "headless": headless,
            "world": world,
            "spawn_robot": "true",
            "spawn_seed": "true",
        }.items(),
    )

    # 2. Khởi chạy Định vị / Dựng bản đồ theo mode (chờ 6.0s để Gazebo và robot sẵn sàng)
    nav_or_mapping_system = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_wh_nav, "launch", "nav.launch.py")
                ),
                launch_arguments={
                    "mode": mode,
                    "use_sim_time": use_sim_time,
                    "autostart": "true",
                }.items(),
            )
        ],
    )

    # 3. Khởi chạy Thị giác máy tính (Tag Detectors + DockServer - chỉ khi navigation, chờ 7.0s)
    vision_system = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_wh_vision, "launch", "visual_servoing.launch.py")
                ),
                launch_arguments={"use_sim_time": use_sim_time}.items(),
                condition=is_nav_mode,
            )
        ],
    )

    # 4. Khởi chạy Điều khiển Tay máy SCARA (PickPlaceServer - chỉ khi navigation, chờ 7.5s)
    manipulation_system = TimerAction(
        period=7.5,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_wh_manip, "launch", "manipulation.launch.py")
                ),
                condition=is_nav_mode,
            )
        ],
    )

    # 5. Khởi chạy Task Manager (CSDL SQLite + FSM - chỉ khi navigation, chờ 8.0s)
    task_manager_system = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_wh_task, "launch", "task_manager.launch.py")
                ),
                condition=is_nav_mode,
            )
        ],
    )

    # 6. Khởi chạy Web Server cục bộ (trong cả 2 chế độ, chờ 8.5s)
    web_system = TimerAction(
        period=8.5,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_wh_web, "launch", "web.launch.py")
                ),
            )
        ],
    )

    # 7. Khởi chạy RViz2 (tùy chọn theo arg rviz, chờ 9.0s)
    rviz_node = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(rviz),
            )
        ],
    )

    # 8. In banner hoàn tất sau khi toàn bộ dịch vụ đã kích hoạt (chờ 10.0s)
    ready_banner = TimerAction(
        period=10.0,
        actions=[OpaqueFunction(function=print_ready_banner)],
    )

    ld = LaunchDescription()
    ld.add_action(declare_mode)
    ld.add_action(declare_headless)
    ld.add_action(declare_rviz)
    ld.add_action(declare_enable_slip)
    ld.add_action(declare_seed_file)
    ld.add_action(declare_world)
    ld.add_action(declare_use_sim_time)

    ld.add_action(warehouse_sim)
    ld.add_action(nav_or_mapping_system)
    ld.add_action(vision_system)
    ld.add_action(manipulation_system)
    ld.add_action(task_manager_system)
    ld.add_action(web_system)
    ld.add_action(rviz_node)
    ld.add_action(ready_banner)

    return ld
