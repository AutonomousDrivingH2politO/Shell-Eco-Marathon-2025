from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='juno_aug',
            executable='obstacle_avoidance.py',
            name='obstacle_avoidance',
            output='screen'
        ),
        Node(
            package='juno_aug',
            executable='steering_brake_node.py',
            name='steering_brake_node',
            output='screen'
        ),
    ])
