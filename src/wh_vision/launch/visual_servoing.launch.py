#!/usr/bin/env python3
"""visual_servoing.launch.py – Khởi chạy detector thị giác và DockServer action server."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_vision = get_package_share_directory("wh_vision")

    marker_sizes_file = os.path.join(pkg_vision, "config", "marker_sizes.yaml")
    servo_params_file = os.path.join(pkg_vision, "config", "servo_params.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Sử dụng thời gian mô phỏng"
    )

    tag_detector_node = Node(
        package="wh_vision",
        executable="tag_detector_node",
        name="tag_detector_dock",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"camera_name": "dock"},
            {"image_topic": "/dock_camera/image_raw"},
            {"camera_info_topic": "/dock_camera/camera_info"},
            {"window_size": 10},
            {"max_reproj_px": 1.0},
            {"config_file": marker_sizes_file},
        ]
    )

    tag_detector_hand_node = Node(
        package="wh_vision",
        executable="tag_detector_node",
        name="tag_detector_hand",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"camera_name": "hand"},
            {"image_topic": "/hand_camera/image_raw"},
            {"camera_info_topic": "/hand_camera/camera_info"},
            {"window_size": 10},
            {"max_reproj_px": 1.0},
            {"config_file": marker_sizes_file},
        ]
    )

    dock_server_node = Node(
        package="wh_vision",
        executable="dock_server",
        name="dock_server",
        output="screen",
        parameters=[
            servo_params_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        tag_detector_node,
        tag_detector_hand_node,
        dock_server_node,
    ])

