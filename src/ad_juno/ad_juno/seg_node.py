#!/home/bylogix/torch/bin/python
import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import torch
import cv2
from pathlib import Path
from cv_bridge import CvBridge

from shared_objects.ROS_utils import Topics, SHOW
from shared_objects.utils_model import preprocessing_image, preprocessing_mask
from ultralytics import YOLO

# Initialize Topics and parameters
topics = Topics()
topic_names = topics.topic_names

# Global parameters
wheelbase = 1.6
model_type = "hybridnets"  # Choose from "hybridnets", "yolop"
half = False
count = 0
seg_img_id = 0

print(os.getcwd())  # Display the current working directory


def initialize_model(model_type, half=False):
    """Initialize and return the segmentation model based on model type."""
    if model_type == "hybridnets":
        model = torch.hub.load('datvuthanh/hybridnets', 'hybridnets', pretrained=True,
                               device='cuda:0' if torch.cuda.is_available() else 'cpu').eval()
    elif model_type == "yolop":
        work_dir= Path(__file__).parent
        model_path = work_dir/"yolopv2.pt"
        device = "cuda" if torch.cuda.is_available() else "cpu"
        model = torch.jit.load(model_path, map_location=device)
    else:
        raise ValueError("Model type not found")
    if half:
        model.half()
    return model


def get_segmentation(model, input_tensor, show=False):
    """Run segmentation on the input tensor and optionally display the segmentation mask."""
    with torch.no_grad():
        if model_type == 'hybridnets' or model_type == "local_hybridnets":
            _, _, cls, _, seg = model(input_tensor)
        elif model_type == "yolop":
            _, seg, _ = model(input_tensor)
        else:
            raise ValueError("Model type not found")
    if show:
        display = seg[0].cpu().numpy()
        cv2.imshow("Segmentation Mask", display)
        cv2.waitKey(1)
    return seg


class SegNode(Node):
    """ROS2 Node for segmentation with periodic image processing and model loading."""

    def __init__(self):
        super().__init__('seg_node')
        self.bridge = CvBridge()
        self.model = initialize_model(model_type, half=half)
        self.bool_msg = Bool()
        self.bool_msg.data = True
        self.count = 0

        # Publishers
        self.seg_img_pub = self.create_publisher(Image, topic_names['segmented_image'], 10)
        self.model_enable_pub = self.create_publisher(Bool, topic_names["model_enable"], 10)

        # Subscribers
        self.create_subscription(Image, topic_names["RGB_image"], self.image_callback, 10)

        # Notify model initialization
        self.model_enable_pub.publish(self.bool_msg)
        self.get_logger().info("Segmentation Node Initialized and Model Enabled")

    def image_callback(self, data):
        """Callback to process images from the RGB image topic."""
        self.count += 1
        if self.count % 9 != 0:
            return

        cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
        input_tensor = preprocessing_image(cv_image, half=half)

        with torch.no_grad():
            start = time.time()
            seg = get_segmentation(self.model, input_tensor)
            self.get_logger().info(f"Segmentation processing time: {time.time() - start}s")

        mask = preprocessing_mask(seg, show=SHOW, improve=True)
        seg_img_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
        self.seg_img_pub.publish(seg_img_msg)
        self.get_logger().info("Segmented image published")


def main(args=None):
    rclpy.init(args=args)
    seg_node = SegNode()

    try:
        rclpy.spin(seg_node)
    except KeyboardInterrupt:
        seg_node.get_logger().info("Segmentation Node shutting down.")

    seg_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
