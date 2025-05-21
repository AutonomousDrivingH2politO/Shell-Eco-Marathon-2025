import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ad_juno',
            executable='enable_node.py',
            name='enable_node',
            output='screen'
        ),
        Node(
            package='ad_juno',
            executable='steering_brake_node.py',
            name='steering_brake_node',
            output='screen'
        ),
        Node(
            package='ad_juno',
            executable='fake_node.py',
            name='path_planning',
            output='screen'
        ),
    ])
