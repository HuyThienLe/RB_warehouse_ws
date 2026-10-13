#!/usr/bin/env python3
"""nav.launch.py – Khởi chạy hệ thống Định vị / Dựng bản đồ, Twist Mux và Collision Monitor.

Hỗ trợ 2 chế độ (Bài 8b.1):
  mode:="navigation" (mặc định):
    - map_server + AMCL + Nav2 (controller, planner, bt_navigator...) + auto_initial_pose
  mode:="mapping":
    - slam_toolbox (online_async) dựng bản đồ thời gian thực
    - KHÔNG chạy AMCL, KHÔNG chạy map_server (tránh xung đột TF map->odom)
Chuỗi điều khiển vận tốc:
  teleop / nav / servo -> twist_mux -> collision_monitor -> /cmd_vel
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_wh_nav = get_package_share_directory("wh_navigation")

    # Đường dẫn cấu hình
    default_map = os.path.join(pkg_wh_nav, "maps", "warehouse_map.yaml")
    default_nav2_params = os.path.join(pkg_wh_nav, "config", "nav2_params.yaml")
    default_slam_params = os.path.join(pkg_wh_nav, "config", "slam_toolbox_params.yaml")
    default_twist_mux_params = os.path.join(pkg_wh_nav, "config", "twist_mux.yaml")
    default_collision_monitor_params = os.path.join(pkg_wh_nav, "config", "collision_monitor_params.yaml")

    # Tham số launch
    use_sim_time = LaunchConfiguration("use_sim_time")
    mode = LaunchConfiguration("mode")
    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock"
    )
    declare_mode_cmd = DeclareLaunchArgument(
        "mode", default_value="navigation", description="Operating mode: navigation | mapping"
    )
    declare_map_cmd = DeclareLaunchArgument(
        "map", default_value=default_map, description="Full path to map yaml file"
    )
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file", default_value=default_nav2_params, description="Full path to nav2 param file"
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="true", description="Automatically startup lifecycle nodes"
    )

    # Điều kiện chế độ
    is_nav_mode = IfCondition(PythonExpression(["'", mode, "' in ('navigation', 'nav')"]))
    is_mapping_mode = IfCondition(PythonExpression(["'", mode, "' in ('mapping', 'slam')"]))

    # Danh sách node vòng đời cho chế độ navigation
    nav_lifecycle_nodes = [
        "map_server",
        "amcl",
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
        "collision_monitor",
    ]

    # ── Chế độ Navigation: Map Server & AMCL & Nav2 ──────────────────────────
    start_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        condition=is_nav_mode,
        parameters=[{"use_sim_time": True, "yaml_filename": default_map}],
    )

    start_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        condition=is_nav_mode,
        parameters=[default_nav2_params],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    start_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        condition=is_nav_mode,
        parameters=[default_nav2_params],
        remappings=[("cmd_vel", "/cmd_vel_nav")],
    )

    start_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        condition=is_nav_mode,
        parameters=[default_nav2_params],
    )

    start_behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        condition=is_nav_mode,
        parameters=[default_nav2_params],
    )

    start_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        condition=is_nav_mode,
        parameters=[default_nav2_params],
    )

    start_waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        condition=is_nav_mode,
        parameters=[default_nav2_params],
    )

    start_velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        condition=is_nav_mode,
        parameters=[default_nav2_params],
        remappings=[
            ("cmd_vel", "/cmd_vel_nav"),
            ("cmd_vel_smoothed", "/cmd_vel_smoothed"),
        ],
    )

    start_lifecycle_manager_nav = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        condition=is_nav_mode,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": nav_lifecycle_nodes},
        ],
    )

    start_auto_initial_pose = Node(
        package="wh_navigation",
        executable="auto_initial_pose.py",
        name="auto_initial_pose",
        output="screen",
        condition=is_nav_mode,
        parameters=[{"use_sim_time": use_sim_time, "x": -2.4, "y": 0.0, "yaw": 0.0}],
    )

    # ── Chế độ Mapping: SLAM Toolbox (online_async) ──────────────────────────
    start_slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        condition=is_mapping_mode,
        parameters=[
            default_slam_params,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    start_lifecycle_manager_mapping = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_mapping",
        output="screen",
        condition=is_mapping_mode,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": ["collision_monitor"]},
        ],
    )

    # ── Dùng chung trong cả 2 chế độ: Twist Mux, Collision Monitor, E-Stop, Mode Pub ─
    start_twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[default_twist_mux_params],
        remappings=[("cmd_vel_out", "/cmd_vel_muxed")],
    )

    start_collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        parameters=[default_collision_monitor_params],
    )

    start_estop_safety = Node(
        package="wh_navigation",
        executable="estop_safety_node.py",
        name="estop_safety_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    start_mode_publisher = Node(
        package="wh_navigation",
        executable="mode_publisher.py",
        name="mode_publisher",
        output="screen",
        parameters=[{"mode": mode}],
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)

    # Navigation nodes
    ld.add_action(start_map_server)
    ld.add_action(start_amcl)
    ld.add_action(start_controller_server)
    ld.add_action(start_planner_server)
    ld.add_action(start_behavior_server)
    ld.add_action(start_bt_navigator)
    ld.add_action(start_waypoint_follower)
    ld.add_action(start_velocity_smoother)
    ld.add_action(start_lifecycle_manager_nav)
    ld.add_action(start_auto_initial_pose)

    # Mapping nodes
    ld.add_action(start_slam_toolbox)
    ld.add_action(start_lifecycle_manager_mapping)

    # Common nodes
    ld.add_action(start_twist_mux)
    ld.add_action(start_collision_monitor)
    ld.add_action(start_estop_safety)
    ld.add_action(start_mode_publisher)

    return ld
