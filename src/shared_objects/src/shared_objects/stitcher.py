import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class Zed2iStitchNode(Node):
    def __init__(self):
        super().__init__('zed2i_stitch_node')
        self.bridge = CvBridge()

        # Load video files instead of ZED2i camera
        self.cap_left = cv2.VideoCapture('/home/bylogix/Shell-Eco-Marathon-2025/calibration_setup/20250521_122720_left_20250521_122720.mp4')
        self.cap_right = cv2.VideoCapture('/home/bylogix/Shell-Eco-Marathon-2025/calibration_setup/20250521_122720_right_20250521_122720.mp4')

        if not self.cap_left.isOpened() or not self.cap_right.isOpened():
            self.get_logger().error('Failed to open one or both video files')
            rclpy.shutdown()
            return

        # Publisher
        self.pub_stitched = self.create_publisher(Image, '/zed2i/stitched', 1)

        # ORB detector and matcher
        self.orb = cv2.ORB_create(nfeatures=2000, scaleFactor=1.2, nlevels=8)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Use FPS from video (or default to 15 if can't detect)
        fps = self.cap_left.get(cv2.CAP_PROP_FPS)
        fps = fps if fps > 0 else 15
        self.timer = self.create_timer(1.0 / 15, self.timer_callback)

        self.get_logger().info('ZED2i Stitch node started using video files.')

    def timer_callback(self):
        ret_l, left_img = self.cap_left.read()
        ret_r, right_img = self.cap_right.read()

        if not ret_l or not ret_r:
            self.get_logger().info('End of video file(s)')
            rclpy.shutdown()
            return

        # Stitch images
        stitched = self.stitch_images(left_img, right_img)
        if stitched is None:
            try:
                ros_img = self.bridge.cv2_to_imgmsg(left_img, encoding='bgr8')
                ros_img.header = Header()
                ros_img.header.stamp = self.get_clock().now().to_msg()
                self.get_logger().info('Publishing left stitch failed!')
                self.pub_stitched.publish(ros_img)
            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge Error (fallback): {e}')
            return

        # Publish stitched image
        try:
            ros_img = self.bridge.cv2_to_imgmsg(stitched, encoding='bgr8')
            ros_img.header = Header()
            ros_img.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().info('Publishing stitch!')
            self.pub_stitched.publish(ros_img)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

    def stitch_images(self, left_img: np.ndarray, right_img: np.ndarray) -> np.ndarray:
        gray_left = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

        kp1, des1 = self.orb.detectAndCompute(gray_left, None)
        kp2, des2 = self.orb.detectAndCompute(gray_right, None)
        if des1 is None or des2 is None:
            self.get_logger().error('No descriptors found')
            return None

        matches = sorted(self.bf.match(des1, des2), key=lambda m: m.distance)
        good = matches[:int(len(matches) * 0.15)]
        if len(good) < 4:
            self.get_logger().error('Not enough good matches')
            return None

        pts1 = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        pts2 = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        H, mask = cv2.findHomography(pts2, pts1, cv2.RANSAC, 5.0)
        if H is None:
            self.get_logger().error('Homography failed')
            return None

        h1, w1 = left_img.shape[:2]
        h2, w2 = right_img.shape[:2]
        pano_w, pano_h = w1 + w2, max(h1, h2)

        pano = cv2.warpPerspective(right_img, H, (pano_w, pano_h))
        pano[0:h1, 0:w1] = left_img

        gray = cv2.cvtColor(pano, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
            return pano[y:y + h, x:x + w]
        return pano

    def destroy_node(self):
        self.cap_left.release()
        self.cap_right.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Zed2iStitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

