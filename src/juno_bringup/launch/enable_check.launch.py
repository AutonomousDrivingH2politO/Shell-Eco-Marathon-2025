import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="ad_juno",
            name="steering_brake_node",
            executable="steering_brake_node",

            output="screen"
        ),
        launch_ros.actions.Node(
            package="ad_juno",
            name="can_node",
            executable="can_node_resurrections",
            output="screen"
        ),
    ])
