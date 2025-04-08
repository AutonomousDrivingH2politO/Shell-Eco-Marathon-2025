#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
# import RPi.GPIO as GPIO
import Jetson.GPIO as GPIO  # Uncomment this for Jetson
import time
from shared_objects.ROS_utils import Topics

class ButtonEngageNode(Node):
    def __init__(self):
        super().__init__('button_engage_node')

        self.button_pin = 24
        self.setup_gpio()

        self.system_engaged = False

        topics = Topics()
        self.topic_names = topics.topic_names

        #Publishers
        self.model_enable_pub = self.create_publisher(Bool, self.topic_names["model_enable"], 1)
        self.engine_enable_pub = self.create_publisher(Bool, self.topic_names["engine_enable"], 1)
        self.stop_enable_pub = self.create_publisher(Bool, self.topic_names["stop_enable"], 1)


        self.create_timer(0.05, self.check_button)

        self.update_system_state()

        self.get_logger().info(f'Button Engage Node started. System will be active while button on pin {self.button_pin} is pressed.')

    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)# 40 PIN Header / GPIO.BOARD
        GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def check_button(self):
        # Check current button state
        button_pressed = GPIO.input(self.button_pin) == GPIO.HIGH

        # Only update if state has changed
        if button_pressed != self.system_engaged:
            self.system_engaged = button_pressed
            self.update_system_state()

    def update_system_state(self):
        enable_msg = Bool()
        enable_msg.data = self.system_engaged

        # Publish to all enable topics
        self.model_enable_pub.publish(enable_msg)
        self.engine_enable_pub.publish(enable_msg)
        self.stop_enable_pub.publish(enable_msg)

        status = "ENGAGED" if self.system_engaged else "DISENGAGED"
        self.get_logger().info(f'Self-driving system {status}')

    def on_shutdown(self):
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args, context=rclpy.get_default_context())
    node = ButtonEngageNode()

    try:
        rclpy.spin(node, executor=rclpy.get_global_executor())
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()