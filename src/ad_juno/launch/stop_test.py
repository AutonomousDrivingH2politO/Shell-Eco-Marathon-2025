from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Declare the "offline" argument
    return LaunchDescription([
        DeclareLaunchArgument('offline', default_value='false', description='Offline mode'),

        # Node for steering_brake_node
        Node(
            package='juno_aug',
            executable='steering_brake_node.py',
            name='steering_brake_node',
            output='screen'
        ),

        # Node for stop_node
        Node(
            package='juno_aug',
            executable='stop_node.py',
            name='stop_node',
            output='screen'
        ),

        # Node for can_node
        Node(
            package='juno_aug',
            executable='can_node_resurrections.py',
            name='can_node',
            output='screen'
        ),

        # Include realsense2 camera launch file unless offline argument is true
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_camera.launch')),
            condition=UnlessCondition(LaunchConfiguration('offline')),
            launch_arguments={
                'align_depth': 'true',
                'linear_accel_cov': '1.0',
                'unite_imu_method': 'linear_interpolation'
            }.items()
        )
    ])
