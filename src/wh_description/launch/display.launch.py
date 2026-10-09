import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory("wh_description")
    
    default_xacro_path = os.path.join(pkg_share, "urdf", "robot.urdf.xacro")
    default_params_file = os.path.join(pkg_share, "config", "robot_params.yaml")
    default_rviz_config = os.path.join(pkg_share, "rviz", "robot.rviz")
    
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    
    # Process xacro
    doc = xacro.process_file(
        default_xacro_path,
        mappings={"params_file": default_params_file}
    )
    robot_desc = doc.toxml()
    
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_desc,
            "use_sim_time": use_sim_time
        }]
    )
    
    jsp_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz_config] if os.path.exists(default_rviz_config) else []
    )
    
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        rsp_node,
        jsp_gui_node,
        rviz_node
    ])
