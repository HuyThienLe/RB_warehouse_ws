#!/usr/bin/env python3
"""warehouse_nav.launch.py – Tích hợp khởi chạy đồng bộ:
1. Môi trường kho Gazebo + Robot AMR (wh_gazebo/warehouse.launch.py)
2. Hệ thống định vị AMCL, Nav2, Twist Mux, Collision Monitor (wh_navigation/nav.launch.py)
3. Hệ thống thị giác ArUco & Visual Servoing PBVS (wh_vision/visual_servoing.launch.py)
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_wh_gazebo = get_package_share_directory("wh_gazebo")
    pkg_wh_nav = get_package_share_directory("wh_navigation")
    pkg_wh_vision = get_package_share_directory("wh_vision")

    headless = LaunchConfiguration("headless", default="false")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    declare_headless = DeclareLaunchArgument(
        "headless", default_value="false", description="Run gazebo headless"
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use sim time"
    )

    # 1. Khởi chạy mô phỏng kho và robot
    warehouse_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_wh_gazebo, "launch", "warehouse.launch.py")
        ),
        launch_arguments={
            "headless": headless,
            "spawn_robot": "true",
            "spawn_seed": "true",
        }.items(),
    )

    # 2. Khởi chạy Nav2 sau khi Gazebo và robot đã sẵn sàng (chờ 6 giây)
    nav2_system = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_wh_nav, "launch", "nav.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": "true",
                }.items(),
            )
        ],
    )

    # 3. Khởi chạy Thị giác & DockServer sau khi Robot state & TF sẵn sàng (chờ 7 giây)
    vision_system = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_wh_vision, "launch", "visual_servoing.launch.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                }.items(),
            )
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_headless)
    ld.add_action(declare_use_sim_time)
    ld.add_action(warehouse_sim)
    ld.add_action(nav2_system)
    ld.add_action(vision_system)
    return ld
