import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='juno_aug',
            executable='steering_brake_node.py',
            name='steering_brake_node',
            output='screen'
        ),
        Node(
            package='juno_aug',
            executable='path_planning.py',
            name='path_planning',
            output='screen'
        ),
        Node(
            package='juno_aug',
            executable='seg_node.py',
            name='seg_node',
            output='screen'
        ),
        Node(
            package='juno_aug',
            executable='can_node_revolutions.py',
            name='can_node',
            output='screen'
        ),
        Node(
            package='juno_aug',
            executable='throttle_node.py',
            name='throttle_node',
            output='screen'
        ),
    ])
