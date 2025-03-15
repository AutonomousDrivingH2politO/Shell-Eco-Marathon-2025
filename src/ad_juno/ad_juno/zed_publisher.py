import rclpy
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import Image

class ZedTest(Node):

    def __init__(self):
        super().__init__('zed_publisher')

        # Set up QoS (Quality of Service)
        video_qos = qos.QoSProfile(depth=10)
        video_qos.reliability = qos.ReliabilityPolicy.RELIABLE
        video_qos.durability = qos.DurabilityPolicy.VOLATILE

        # Create subscribers for right and left images
        self.right_sub = self.create_subscription(
            Image,
            'right_image',
            self.image_right_rectified_callback,
            video_qos
        )
        self.left_sub = self.create_subscription(
            Image,
            'left_image',
            self.image_left_rectified_callback,
            video_qos
        )

    def image_right_rectified_callback(self, msg):
        self.get_logger().info(
            f"Right Rectified image received from ZED\t"
            f"Size: {msg.width}x{msg.height} - "
            f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec} sec"
        )

    def image_left_rectified_callback(self, msg):
        self.get_logger().info(
            f"Left Rectified image received from ZED\t"
            f"Size: {msg.width}x{msg.height} - "
            f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec} sec"
        )

def main(args=None):
    rclpy.init(args=args)

    # Create node instance
    node = ZedTest()
    
    # Spin to keep the node active
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Shutdown the node once done
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

