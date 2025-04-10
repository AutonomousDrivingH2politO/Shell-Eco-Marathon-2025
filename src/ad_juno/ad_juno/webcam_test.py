# !/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from shared_objects.ROS_utils import Topics


class WebcamPublisher(Node):

    def __init__(self):
        super().__init__('webcam_publisher')
        # Queue size 10 topic with name /camera/camera/color/image_raw
        self.publisher_ =  self.create_publisher(Image,'/zed/zed_node/rgb/image_rect_color' ,1 )
        self.brigde = CvBridge()
        self.path = "/home/bylogix/Shell-Eco-Marathon-2025/src/test_files/video_cerrina1.webm"
        self.capture = cv2.VideoCapture(self.path)

        timer_period = 0.1 # 2 images per second

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.capture.read()

        if not ret:
            self.get_logger().error('Failed to capture image')
            return
        ros_img = self.brigde.cv2_to_imgmsg(frame , encoding='bgr8')
        self.publisher_.publish(ros_img)
        self.get_logger().info('Publishing image')

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
           node.get_logger().info('Keyboard interrupt signal, shutting down node.')     
        
    finally:
        node.capture.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()