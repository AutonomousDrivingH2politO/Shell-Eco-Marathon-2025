import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ad_juno',
            executable='steering_brake_node',
            name='steering_brake_node',
            output='screen'
        ),
        Node(
            package='ad_juno',
            executable='path_planning',
            name='path_planning',
            output='screen'
        ),
        Node(
            package='ad_juno',
            executable='seg_node',
            name='seg_node',
            output='screen'
        ),
        Node(
            package='ad_juno',
            executable='can_node_resurrections',
            name='can_node',
            output='screen'
        ),
        Node(
            package='ad_juno',
            executable='throttle_node',
            name='throttle_node',
            output='screen'
        ),
    ])
