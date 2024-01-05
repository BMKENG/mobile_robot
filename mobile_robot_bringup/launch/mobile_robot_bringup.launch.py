#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess

def generate_launch_description():

    mcu_node = Node(
        package='mobile_robot_bringup',
        executable='mobile_robot_serial',
        output='screen',
    )
    #  LiDAR 노드
    # lidar_node = Node(
    
    odom_to_base_publisher = ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "tf2_ros",
            "static_transform_publisher",
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "odom",
            "base_link",
        ],
        output="screen",
    )
    return LaunchDescription([
        mcu_node,
        odom_to_base_publisher
    ])
