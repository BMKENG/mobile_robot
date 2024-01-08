#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    mcu_node = Node(
        package='mobile_robot_bringup',
        executable='mobile_robot_serial',
        output='screen',
    )
    #  LiDAR 노드
    # lidar_node = Node(
    base_to_laser_publisher = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            "0",
            "0",
            "0.1",
            "-3.141592",
            "0",
            "0",
            "base_link",
            "laser",
        ],
        output="screen",
    )
    g2lidar_prefix = get_package_share_directory("ydlidar_ros2_driver")
    start_g2lidar_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(g2lidar_prefix, "launch", "ydlidar_launch.py"))
    )
    start_robot_description_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bmk_mobile_description"), "launch", "display.launch.py"

            )
        )
    )
    return LaunchDescription([
        mcu_node,
        start_robot_description_cmd,
        start_g2lidar_cmd,
        base_to_laser_publisher,
    ])
