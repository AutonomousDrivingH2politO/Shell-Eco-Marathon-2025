from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("zed_wrapper"),
                                "launch",
                                "zed_camera.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={"camera_model": "zed2i", "sim_mode": "true"}.items(),
            ),
           Node(
                package="ad_juno",
                executable="obstacle_avoidance",
                name="obstacle_avoidance",
                output="screen",
            ),
            Node(
                package="ad_juno",
                executable="steering_brake_node",
                name="steering_brake_node",
                output="screen",
            ),
            Node(
                package="ad_juno",
                executable="seg_node",
                name="seg_node",
                output="screen",
            ),
            Node(
                package="ad_juno",
                executable="can_node_resurrections",
                name="can_node",
                output="screen",
            ),
            Node(
                package="ad_juno",
                executable="throttle_node",
                name="throttle_node",
                output="screen",
            ),
            Node(
                package="ad_juno",
                executable="stop_node",
                name="stop_node",
                output="screen",
            ),
        ]
    )
