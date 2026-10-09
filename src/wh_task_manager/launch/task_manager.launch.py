#!/usr/bin/env python3
"""task_manager.launch.py – Khởi động hệ thống quản lý nhiệm vụ và CSDL kho hàng (Bài 7)."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_db_path = DeclareLaunchArgument(
        "db_path",
        default_value="",
        description="Đường dẫn file SQLite database (để trống để dùng ~/.warehouse/inventory.db)",
    )

    task_manager_node = Node(
        package="wh_task_manager",
        executable="task_manager_node",
        name="task_manager_node",
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        declare_db_path,
        task_manager_node,
    ])
