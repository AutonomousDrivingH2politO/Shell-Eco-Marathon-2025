from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ad_juno',
            executable='obstacle_avoidance',
            name='obstacle_avoidance',
            output='screen'
        ),
        Node(
            package='ad_juno',
            executable='steering_node',
            name='steering_node',
            output='screen'
        ),
    ])
