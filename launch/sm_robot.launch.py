from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cmeresearch_robot_state',
            executable='sm_robot_node',
            name='sm_robot',
            output='screen',
            parameters=[
                # {'my_parameter': 'value'}
            ]
        )
    ])
