#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


from std_msgs.msg import Float64, Bool
import time
#from std_msgs.msg import Float32

from shared_objects.Motor import Stepper
from shared_objects.ROS_utils import Topics, SHOW

topics=Topics()
topic_names=topics.topic_names

class SteeringBrakeNode(Node):
    def __init__(self):
        # initializing node
        super().__init__('steering_brake_node')
        self.get_logger().info("----Steering and Brake node has been started----")

        # predefined constants
        self.SIZE=10
        self.TRANSMISSION_RATIO= 15
        self.MAX_ANGLE=14

        # initializing topics
        self.topics=Topics()
        self.topic_names=topics.topic_names

        modules={
            "steering":3,
            "brake":1
        }

        # setup CAN bus
        self.steering=Stepper(
                 interface="can",
                 module_id=modules["steering"],
                 max_acc=50_000,
                 max_velocity=80_000,
                 V1=100_000
                 )

        self.brake=Stepper(
                    interface="can",
                    module_id=modules["brake"],
                    max_acc=140_000,
                    max_velocity=50_000,
                    V1=0
                    )
        # initialize subscribers
        self.steering_sub = self.create_subscription(
            Float64,
            self.topic_names["steering"],
            self.callback_stepangle,
            10
        )

        self.brake_sub = self.create_subscription(
            Bool,
            self.topic_names["stop"],
            self.callback_brake,
            10
        )

        # create publishers
        self.stop_pub = self.create_publisher(
            Bool,
            self.topic_names["stop"],
            1
        )

        self.enable_pub = self.create_publisher(
            Bool,
            self.topic_names["stop_enable"],
            1
        )

        self.get_logger().info('Steering and Brake node initialized')

    def callback_stepangle(self, msg):
        MAX_ANGLE=14 # pre-defined
        angle = msg.data
        self.get_logger().info(f'Received angle: {angle}')

        step_angle=1.8 # the reason is 360/200

        # applying steering limits and calculating step
        if angle > self.MAX_ANGLE:
            step = int((self.MAX_ANGLE/self.STEP_ANGLE)*256)
        elif angle < -self.MAX_ANGLE:
            step = int(-(self.MAX_ANGLE/self.STEP_ANGLE)*256)
        else:
            step = int((angle/self.STEP_ANGLE)*256)

        self.get_logger().info(f'Calculated steps: {step}')
        self.steering.move_stepper(-int(step*self.TRANSMISSION_RATIO))
        self.get_logger().info('Finished moving')

    def callback_brake(self, msg):
        if msg.data:
            self.brake.brake()

            # publish brake status
            bool_msg = Bool()
            bool_msg.data = False
            self.stop_pub.publish(bool_msg)
            self.get_logger().info('Finished braking')

            # wait
            time.sleep(10)
            bool_msg.data = True
            self.enable_pub.publish(bool_msg)



def main(args = None):
    rclpy.init(args=args)  # initialize ROS2 context
    node = SteeringBrakeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down node')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
