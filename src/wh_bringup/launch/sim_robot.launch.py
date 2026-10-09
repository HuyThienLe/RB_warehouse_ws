import os
import re
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_wh_desc = get_package_share_directory("wh_description")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    
    gui = LaunchConfiguration("gui", default="true")
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    
    # Files
    xacro_file = os.path.join(pkg_wh_desc, "urdf", "robot.urdf.xacro")
    params_file = os.path.join(pkg_wh_desc, "config", "robot_params.yaml")
    arm_controllers_file = os.path.join(pkg_wh_desc, "config", "arm_controllers.yaml")
    ekf_config_file = os.path.join(pkg_wh_desc, "config", "ekf.yaml")
    
    # Process xacro without XML declaration prolog to prevent rcl parameter parser errors
    doc = xacro.process_file(
        xacro_file,
        mappings={
            "params_file": params_file,
            "controllers_file": arm_controllers_file
        }
    )
    raw_xml = doc.documentElement.toxml()
    clean_xml = re.sub(r"<!--.*?-->", "", raw_xml, flags=re.DOTALL)
    
    # 1. Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "gui": gui,
            "server_required": "true",
            "gui_required": "false"
        }.items()
    )
    
    # 2. Robot State Publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": clean_xml,
            "use_sim_time": True
        }]
    )
    
    # 3. Spawn Robot in Gazebo
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "warehouse_robot",
            "-x", "0.0",
            "-y", "0.0",
            "-z", "0.01"
        ],
        output="screen"
    )
    
    # 4. ros2_control Controller Spawners
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
    
    delay_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )
    
    delay_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner]
        )
    )
    
    # 5. Helper nodes: Mecanum wheel state, Slip odometry, EKF, Gripper
    mecanum_node = Node(
        package="wh_description",
        executable="mecanum_wheel_state_node.py",
        name="mecanum_wheel_state_node",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )
    
    slip_odom_node = Node(
        package="wh_description",
        executable="slip_odom_node.py",
        name="slip_odom_node",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )
    
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config_file, {"use_sim_time": True}]
    )
    
    gripper_node = Node(
        package="wh_description",
        executable="gripper_node.py",
        name="gripper_node",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Start Gazebo GUI"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        gazebo,
        rsp,
        spawn_entity,
        delay_jsb_after_spawn,
        delay_arm_after_jsb,
        mecanum_node,
        slip_odom_node,
        ekf_node,
        gripper_node
    ])
