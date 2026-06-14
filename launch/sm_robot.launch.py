from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmeresearch_robot_state',
            executable='sm_robot_node',
            name='sm_robot',
            output='screen',
        ),
        Node(
            package='cmeresearch_robot_state',
            executable='nav_status_node',
            name='nav_status',
            output='screen',
        ),
        Node(
            package='cmeresearch_robot_state',
            executable='system_stats_node',
            name='system_stats',
            output='screen',
        ),
    ])
