from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # robot_state_publisher를 실행하는 노드를 설정합니다.
    mobile_robot_behavior_gui_node = Node(
        package='mobile_robot_gui',
        executable='mobile_robot_behavior_gui',
        name='mobile_robot_behavior_gui_node',
        output='log'
    )
    mobile_robot_behavior_client_gui_node = Node(
        package='mobile_robot_gui',
        executable='mobile_robot_behavior_client_gui',
        name='mobile_robot_behavior_client_gui_node',
        output='log'
    )

    return LaunchDescription([
        mobile_robot_behavior_gui_node,
        mobile_robot_behavior_client_gui_node,
    ])
