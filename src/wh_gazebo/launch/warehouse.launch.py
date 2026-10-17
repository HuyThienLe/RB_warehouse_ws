"""warehouse.launch.py – Khởi động Gazebo với world kho, robot AMR và box_manager.

Tham số:
  headless:=true    – chạy gzserver không có GUI
  world:=<path>     – đường dẫn file .world (mặc định: warehouse.world)
  spawn_seed:=true  – thả hộp theo inventory_seed.yaml
  spawn_robot:=true – khởi chạy và thả robot AMR vào trạm HOME
"""

import os
import re
import subprocess
import time as _time
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    RegisterEventHandler,
    AppendEnvironmentVariable,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _kill_stale_gazebo(context, *args, **kwargs):
    """Kill stale gzserver/gzclient đồng bộ trước khi khởi chạy mới."""
    subprocess.run(["pkill", "-9", "gzserver"], capture_output=True)
    subprocess.run(["pkill", "-9", "gzclient"], capture_output=True)
    _time.sleep(1.0)
    return []


def generate_launch_description():
    pkg_wh_gazebo = get_package_share_directory("wh_gazebo")
    pkg_wh_desc = get_package_share_directory("wh_description")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    # ── Đường dẫn world & models ─────────────────────────────────────────────
    models_dir = os.path.join(pkg_wh_gazebo, "models")
    textures_dir = os.path.join(
        models_dir, "aruco_tags", "materials", "textures"
    )
    world_file = os.path.join(pkg_wh_gazebo, "worlds", "warehouse.world")

    # ── Đường dẫn Robot URDF & Config ────────────────────────────────────────
    xacro_file = os.path.join(pkg_wh_desc, "urdf", "robot.urdf.xacro")
    robot_params_file = os.path.join(pkg_wh_desc, "config", "robot_params.yaml")
    arm_controllers_file = os.path.join(pkg_wh_desc, "config", "arm_controllers.yaml")
    ekf_config_file = os.path.join(pkg_wh_desc, "config", "ekf.yaml")

    # Xử lý Xacro robot description
    doc = xacro.process_file(
        xacro_file,
        mappings={
            "params_file": robot_params_file,
            "controllers_file": arm_controllers_file
        }
    )
    raw_xml = doc.documentElement.toxml()
    clean_robot_xml = re.sub(r"<!--.*?-->", "", raw_xml, flags=re.DOTALL)

    # ── Arguments ────────────────────────────────────────────────────────────
    headless_arg = DeclareLaunchArgument(
        "headless", default_value="false",
        description="Run without Gazebo GUI"
    )
    world_arg = DeclareLaunchArgument(
        "world", default_value=world_file,
        description="Path to .world file"
    )
    spawn_seed_arg = DeclareLaunchArgument(
        "spawn_seed", default_value="true",
        description="Spawn seed boxes from inventory_seed.yaml"
    )
    spawn_robot_arg = DeclareLaunchArgument(
        "spawn_robot", default_value="true",
        description="Spawn AMR robot into the warehouse world"
    )
    dynamic_obstacle_arg = DeclareLaunchArgument(
        "dynamic_obstacle", default_value="true",
        description="Enable moving dynamic obstacle in warehouse corridor"
    )

    # ── Env vars ──────────────────────────────────────────────────────────────
    current_resource_path = os.environ.get("GAZEBO_RESOURCE_PATH", "")
    resource_paths = ["/usr/share/gazebo-11", models_dir, textures_dir]
    if current_resource_path:
        for p in current_resource_path.split(":"):
            if p and p not in resource_paths:
                resource_paths.append(p)
    new_resource_path = ":".join(resource_paths)

    set_resource_path = SetEnvironmentVariable(
        "GAZEBO_RESOURCE_PATH", new_resource_path
    )
    set_model_path = AppendEnvironmentVariable(
        "GAZEBO_MODEL_PATH", models_dir,
    )

    # ── Kill stale gzserver đồng bộ ──────────────────────────────────────────
    kill_stale = OpaqueFunction(function=_kill_stale_gazebo)

    # ── Gazebo server ─────────────────────────────────────────────────────────
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "verbose": "true",
        }.items(),
    )

    # ── Gazebo client (GUI) ───────────────────────────────────────────────────
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        ),
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )

    # ── Robot Nodes & Spawners ───────────────────────────────────────────────
    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": clean_robot_xml,
            "use_sim_time": True
        }],
        condition=IfCondition(LaunchConfiguration("spawn_robot"))
    )

    # 2. Spawn Robot tại trạm HOME (x = -2.4, y = 0.0, z = 0.02, yaw = 0.0)
    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "warehouse_robot",
            "-x", "-2.4",
            "-y", "0.0",
            "-z", "0.02",
            "-Y", "0.0"
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("spawn_robot"))
    )

    # 3. ros2_control Controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "30"],
        output="screen"
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager-timeout", "30"],
        output="screen"
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager-timeout", "30"],
        output="screen"
    )

    delay_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[joint_state_broadcaster_spawner]
        ),
        condition=IfCondition(LaunchConfiguration("spawn_robot"))
    )

    delay_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner, gripper_controller_spawner]
        ),
        condition=IfCondition(LaunchConfiguration("spawn_robot"))
    )

    # 4. Robot helper nodes: Odometry, Mecanum, EKF, Gripper
    mecanum_node = Node(
        package="wh_description",
        executable="mecanum_wheel_state_node.py",
        name="mecanum_wheel_state_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("spawn_robot"))
    )

    slip_odom_node = Node(
        package="wh_description",
        executable="slip_odom_node.py",
        name="slip_odom_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("spawn_robot"))
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_file, {"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("spawn_robot"))
    )

    gripper_node = Node(
        package="wh_description",
        executable="gripper_node.py",
        name="gripper_node",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("spawn_robot"))
    )

    # ── box_manager node ──────────────────────────────────────────────────────
    box_manager_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="wh_gazebo",
                executable="box_manager",
                name="box_manager",
                output="screen",
                parameters=[
                    {"use_seed": LaunchConfiguration("spawn_seed")}
                ],
            )
        ],
    )

    # 4. Node điều khiển vật cản động tuần tra qua lại hành lang (chờ 4.0s để Gazebo sẵn sàng)
    dynamic_obstacle_system = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="wh_gazebo",
                executable="dynamic_obstacle_node.py",
                name="dynamic_obstacle_controller",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "center_x": -0.40,
                    "patrol_min_y": -0.85,
                    "patrol_max_y": 0.85,
                    "speed": 0.22,
                    "pause_time": 0.8,
                    "height_z": 0.11,
                }],
                condition=IfCondition(LaunchConfiguration("dynamic_obstacle")),
            )
        ],
    )

    return LaunchDescription([
        headless_arg,
        world_arg,
        spawn_seed_arg,
        spawn_robot_arg,
        dynamic_obstacle_arg,
        # Environment
        set_resource_path,
        set_model_path,
        # Kill stale FIRST (synchronous), then launch
        kill_stale,
        gzserver,
        gzclient,
        # Robot
        robot_state_publisher,
        spawn_robot_node,
        delay_jsb_after_spawn,
        delay_arm_after_jsb,
        mecanum_node,
        slip_odom_node,
        ekf_node,
        gripper_node,
        # Box Manager & Dynamic Obstacle
        box_manager_node,
        dynamic_obstacle_system,
    ])
