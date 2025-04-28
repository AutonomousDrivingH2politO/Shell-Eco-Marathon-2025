from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
        # exz
            Node(
                package="ad_juno",
                executable="video_test",
                name="video_test",
                output="screen",
            ),
            Node(
                package="ad_juno",
                executable="path_planning",
                name="path_planning",
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
                executable="steering_node_relative",
                name="steering_node_relative",
                output="screen",
            ),
          
        ]
    )
 