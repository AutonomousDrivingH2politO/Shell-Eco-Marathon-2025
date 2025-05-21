import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import pyzed.sl as sl

class Zed2iStitchNode(Node):
    def __init__(self):
        super().__init__('zed2i_stitch_node')
        self.bridge = CvBridge()

        # Initialize ZED2i camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD720
        init_params.camera_fps = 15
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f'Failed to open ZED2i camera: {err}')
            rclpy.shutdown(context=rclpy.get_default_context())
            return

        # Publisher for stitched panorama (or fallback left image)
        self.pub_stitched = self.create_publisher(Image, '/zed2i/stitched', 10)

        # ORB detector and BFMatcher
        self.orb = cv2.ORB_create(nfeatures=2000, scaleFactor=1.2, nlevels=8)
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Timer to grab and process images at camera FPS
        self.timer = self.create_timer(1.0 / init_params.camera_fps, self.timer_callback)
        self.get_logger().info('ZED2i Stitch node started.')

    def timer_callback(self):
        runtime_params = sl.RuntimeParameters()
        if self.zed.grab(runtime_params) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error('Failed to grab ZED2i frame')
            return

        # Retrieve left and right rectified images
        left_mat = sl.Mat()
        right_mat = sl.Mat()
        self.zed.retrieve_image(left_mat, sl.VIEW.LEFT)
        self.zed.retrieve_image(right_mat, sl.VIEW.RIGHT)

        # Convert to OpenCV BGR
        left_img = cv2.cvtColor(left_mat.get_data(), cv2.COLOR_BGRA2BGR)
        right_img = cv2.cvtColor(right_mat.get_data(), cv2.COLOR_BGRA2BGR)

        # Stitch images
        stitched = self.stitch_images(left_img, right_img)
        if stitched is None:
            # Fallback: publish left image if stitching fails
            try:
                ros_img = self.bridge.cv2_to_imgmsg(left_img, encoding='bgr8')
                ros_img.header = Header()
                ros_img.header.stamp = self.get_clock().now().to_msg()
                self.pub_stitched.publish(ros_img)
            except CvBridgeError as e:
                self.get_logger().error(f'CvBridge Error (fallback): {e}')
            return

        # Publish stitched image
        try:
            ros_img = self.bridge.cv2_to_imgmsg(stitched, encoding='bgr8')
            ros_img.header = Header()
            ros_img.header.stamp = self.get_clock().now().to_msg()
            self.pub_stitched.publish(ros_img)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

    def stitch_images(self, left_img: np.ndarray, right_img: np.ndarray) -> np.ndarray:
        # Convert to grayscale
        gray_left = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

        # Detect ORB keypoints and descriptors
        kp1, des1 = self.orb.detectAndCompute(gray_left, None)
        kp2, des2 = self.orb.detectAndCompute(gray_right, None)
        if des1 is None or des2 is None:
            self.get_logger().error('No descriptors found')
            return None

        # Match and sort
        matches = sorted(self.bf.match(des1, des2), key=lambda m: m.distance)
        good = matches[:int(len(matches) * 0.15)]

        # Keypoint arrays
        pts1 = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1,1,2)
        pts2 = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1,1,2)

        # Compute homography
        H, mask = cv2.findHomography(pts2, pts1, cv2.RANSAC, 5.0)
        if H is None:
            self.get_logger().error('Homography failed')
            return None

        # Dimensions for panorama
        h1, w1 = left_img.shape[:2]
        h2, w2 = right_img.shape[:2]
        pano_w, pano_h = w1 + w2, max(h1, h2)

        # Warp and blend
        pano = cv2.warpPerspective(right_img, H, (pano_w, pano_h))
        pano[0:h1,0:w1] = left_img

        # Crop borders
        gray = cv2.cvtColor(pano, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray,1,255,cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            x,y,w,h = cv2.boundingRect(max(contours, key=cv2.contourArea))
            return pano[y:y+h,x:x+w]
        return pano

    def destroy_node(self):
        self.zed.close()
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
