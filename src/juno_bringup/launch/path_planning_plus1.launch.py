from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import  FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("zed_wrapper"),
                                "launch",
                                "zed_camera.launch",
                            ]
                        )
                    ]
                ),
                launch_arguments={"camera_model": "zed2i", "sim_mode": "false"}.items(),
            ),
       Node(
            package='ad_juno',
            executable='obstacle_avoidance',
            name='obstacle_avoidance',
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
