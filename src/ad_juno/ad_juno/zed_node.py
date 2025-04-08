#!/usr/bin/python3

import rclpy
import cv2
import pyzed.sl as sl
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

class ZEDCameraNode(Node):

    def __init__(self):
        super().__init__('zed_node')

        # Initialize ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720  # Use HD720 or HD1200 video mode
        init_params.camera_fps = 15  # Set fps at 15

        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Failed to open the ZED camera: {repr(err)}")
            rclpy.shutdown(context=rclpy.get_default_context())
            return

        # Initialize CvBridge to convert OpenCV images to ROS Image messages
        self.bridge = CvBridge()

        # Create a publisher to publish images
        self.image_pub = self.create_publisher(Image, '/zed/zed_node/rgb/image_rect_color', 1)

        # Set up a timer to grab images periodically
        self.timer = self.create_timer(1.0 / 15, self.timer_callback)

    def timer_callback(self):
        # Grab an image from ZED
        left_image = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()

        if self.zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            # Retrieve the left image
            self.zed.retrieve_image(left_image, sl.VIEW.LEFT)

            # Convert ZED image to OpenCV format (BGRA)
            opencv_image = cv2.cvtColor(left_image.get_data(), cv2.COLOR_BGRA2BGR)

            # Convert OpenCV image to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(opencv_image, encoding="bgr8")

            # Add timestamp to the message header
            ros_image.header = Header()
            ros_image.header.stamp = self.get_clock().now().to_msg()

            # Publish the image
            self.image_pub.publish(ros_image)
            #self.get_logger().info(f"Published image: {left_image.get_width()} x {left_image.get_height()}")

    def close_camera(self):
        self.zed.close()
        super().destroy_node()

def main():
    rclpy.init(args=None, context=rclpy.get_default_context())

    node = ZEDCameraNode()
    try:
        rclpy.spin(node, rclpy.get_global_executor())
    except KeyboardInterrupt:
        pass
    finally:
        node.close_camera()
        rclpy.shutdown(context=rclpy.get_default_context())

if __name__ == '__main__':
    main()

