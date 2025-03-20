from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
  return LaunchDescription([
    Node(
    package="ad_juno",
    executable="button_engage_node",
    name="button_engage_node",
    output="screen",
  ),
  ])
 