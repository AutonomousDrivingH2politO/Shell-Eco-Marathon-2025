#!/usr/bin/env python
"""
The previous names for all the CAN Bus rewrites were named after matrix movies:
can_node
can_node_reloaded
can_node_revolutions
can_node_resurrections

Let's hope there is a new one by the time of the rewrite
"""

import os
import sys
from enum import Enum

import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int16
from shared_objects.ROS_utils import Topics


class CanBus(Node):
    def __init__(self):
        super().__init__("can_node", parameter_overrides=[])
        self.run_mode = 0
        self.rpm = 0
        self.start_time = -1
        self.can_interface = can.interface.Bus(channel="can0", bustype="socketcan")
        self.logger = self.get_logger()

        # Initialize topics and publishers
        self.topics = Topics()
        self.topic_names = self.topics.topic_names
        self.spd_pub = self.create_publisher(Float32, self.topic_names["speed"], 10)
        self.rpm_pub = self.create_publisher(Int16, self.topic_names["rpm"], 10)
        self.engine_enable_pub = self.create_publisher(Bool, self.topic_names["engine_enable"], 10)
        self.create_subscription(Float32, self.topic_names["throttle"], self.callback_throttle, 10)

        # Send enable message on startup
        self.send_can(self.can_ids.ENABLE.value, 65535)
        self.logger.info("Sent enable message")

        # Attach the CAN Listener
        self.listener = CanBus.CanListener(self, self.spd_pub)
        self.notifier = can.Notifier(self.can_interface, [self.listener])

        # ROS 2 shutdown handling
        self.create_timer(0.01, lambda: None)  # Small timer to keep node active
        self.get_logger().info("CAN Node created")
    
    #Enum for CAN Locations 
    #I used enums for typesafety and immutability
    class can_ids(Enum):
        SPEED = 0x602
        EMERGENCY = 0x11
        RPM = 0x600
        THROTTLE = 0x640
        ENABLE = 0x21

    def send_can(self, id: int, data: int):
        data_array = list(data.to_bytes(2, byteorder="big"))
        data_array += [0] * (8 - len(data_array))
        msg = can.Message(arbitration_id=id, data=bytearray(data_array), is_extended_id=False)
        self.logger.info(f"Sending CAN message with ID {msg.id}")
        self.can_interface.send(msg)

    def callback_throttle(self, data: Float32):
        throttle = data.data
        id = self.can_ids.THROTTLE.value
        content = int(round(throttle * 81.92, 2))
        self.send_can(id, content)

    def disable_connection(self):
        self.logger.info("Disabling connection")
        self.send_can(self.can_ids.ENABLE.value, 0)
        self.can_interface.shutdown()

    def butcher_mode(self, message: can.Message):
        em_data = float(message.data[3])
        if em_data != 0:
            self.disable_connection()
            nodes_list = os.popen("ros2 node list").readlines()
            for node in nodes_list:
                node = node.strip()
                os.system(f"ros2 node kill {node}")

    class CanListener(Node, can.Listener):
        def __init__(self, can_bus: "CanBus", spd_pub):
            super().__init__("CANListener", parameter_overrides=[])
            self.can_bus = can_bus  # Reference to the main CanBus instance
            self.spd_pub = spd_pub
            self.logger = self.get_logger()
            self.logger.info("Listener initialized")

        def on_message_received(self, msg: can.Message):
            match msg.arbitration_id:
                case CanBus.can_ids.SPEED.value:
                    # Extract speed data and publish it
                    spd_data = int.from_bytes(msg.data[4:6], byteorder="big") * 0.036
                    self.spd_pub.publish(Float32(data=spd_data))
                    self.logger.info(f"Published speed: {spd_data} km/h")

                case CanBus.can_ids.EMERGENCY.value:
                    # Call butcher_mode on the CanBus instance
                    self.can_bus.butcher_mode(msg)

                case CanBus.can_ids.RPM.value:
                    # Extract RPM data
                    rpm_data = msg.data[0:2]
                    self.can_bus.rpm = int.from_bytes(rpm_data, byteorder="little")
                    self.logger.info(f"Received RPM: {self.can_bus.rpm}")

def main():
    rclpy.init(args=sys.argv)
    can_node = CanBus()
    try:
        rclpy.spin(can_node)
    except KeyboardInterrupt:
        can_node.logger.info("CAN Node interrupted by user")
    finally:
        can_node.disable_connection()
        can_node.notifier.stop()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

