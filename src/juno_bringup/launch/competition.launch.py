from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        # Include path planning launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('juno_bringup'),
                    'launch',
                    'path_planning.launch.py'
                ])
            ])
        ),
        
        # Include obstacle avoidance launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('juno_bringup'),
                    'launch',
                    'obstacle_avoidance.launch.py'
                ])
            ])
        ),
        
        # Add more launch files as needed
    ]) 