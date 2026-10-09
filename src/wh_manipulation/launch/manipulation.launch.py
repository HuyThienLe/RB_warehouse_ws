"""manipulation.launch.py – Khởi động hệ thống điều khiển tay máy SCARA TRR."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_wh_manip = get_package_share_directory("wh_manipulation")
    poses_config = os.path.join(pkg_wh_manip, "config", "arm_poses.yaml")

    pick_place_node = Node(
        package="wh_manipulation",
        executable="pick_place_server",
        name="pick_place_server",
        output="screen",
        parameters=[
            {"poses_config": poses_config},
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        pick_place_node,
    ])
