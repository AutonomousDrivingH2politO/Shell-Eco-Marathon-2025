import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ad_juno',
            executable='enable_node',
            name='enable_node',
            output='screen'
        ),
        Node(
            package='ad_juno',
            executable='steering_brake_node',
            name='steering_brake_node',
            output='screen'
        ),
        Node(
            package='ad_juno',
            executable='fake_node',
            name='path_planning',
            output='screen'
        ),
    ])
