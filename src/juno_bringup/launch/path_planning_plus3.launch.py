from launch import LaunchDescription
from launch.actions import TimerAction,ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # declare all nodes up‐front
    # 2) Start ros2 bag record (all topics) writing into that directory
    bag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-a',                                  # record all topics
            '-o', '/home/bylogix/rosbag'           # output prefix/directory
        ],
        output='screen'
    )

    zed_node = Node(
        package="ad_juno", executable="zed_node", name="zed_node", output="screen"
    )
    seg_node = Node(
        package="ad_juno", executable="seg_node", name="seg_node", output="screen"
    )
    path_planning_node = Node(
        package="ad_juno",
        executable="path_planning",
        name="path_planning",
        output="screen",
    )
    can_node = Node(
        package="ad_juno",
        executable="can_node_resurrections",
        name="can_node",
        output="screen",
    )
    throttle_node = Node(
        package="ad_juno", executable="throttle_node", name="throttle_node", output="screen"
    )
    steering_node = Node(
        package="ad_juno", executable="steering_node", name="steering_node", output="screen"
    )

    # schedule each with a TimerAction, offsetting by 3s each time
    return LaunchDescription([
        bag_record,

        # at t=0
        TimerAction(period=0.0, actions=[zed_node]),
        # at t=3s
        TimerAction(period=3.0, actions=[seg_node]),
        # at t=6s
        TimerAction(period=6.0, actions=[path_planning_node]),
        # at t=9s
        TimerAction(period=9.0, actions=[can_node]),
        # at t=12s
        TimerAction(period=12.0, actions=[throttle_node, steering_node]),
    ])
