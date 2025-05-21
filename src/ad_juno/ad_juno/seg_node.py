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
import numpy
from argparse import Namespace
from shared_objects.ROS_utils import Topics, SHOW
from shared_objects.utils_model import preprocessing_image, preprocessing_image_no_normalisation, preprocessing_mask
from shared_objects.utils_model import TwinLiteNet
from shared_objects.TwinLiteNetPlus.model.model import TwinLiteNetPlus
#from shared_objects.TwinLiteNetPlus.demo import show_seg_result, detect
# from ultralytics import YOLO # YOLO is not used in the provided snippet for segmentation model init

# Initialize Topics and parameters
topics = Topics()
topic_names = topics.topic_names

# Global parameters
wheelbase = 1.6
model_type = "twinplus"  # Choose from "hybridnets", "yolop", "twin", "twinplus"
half = False
count = 0
seg_img_id = 0


def initialize_model(model_type_param, half_param=False): # Renamed to avoid conflict with global
    """Initialize and return the segmentation model based on model type."""
    if model_type_param == "hybridnets":
        model = torch.hub.load('datvuthanh/hybridnets', 'hybridnets', pretrained=True,
                               device='cuda:0' if torch.cuda.is_available() else 'cpu').eval()
    elif model_type_param == "yolop":
        #work_dir= Path(__file__).resolve().parent # Use resolve() for robustness
        model_path = "/home/bylogix/Downloads/yolopv2.pt"
        device = "cuda" if torch.cuda.is_available() else "cpu"
        model = torch.jit.load(model_path, map_location=device)
    elif model_type_param == "twin":
        model = TwinLiteNet()
        device = "cuda" if torch.cuda.is_available() else "cpu"
        model = torch.nn.DataParallel(model)
        model.load_state_dict(torch.load('/home/bylogix/TwinLiteNet/pretrained/best.pth', map_location=device))
        model = model.to(device)
        model.eval()
    elif model_type_param== "twinplus":
        device = "cuda" if torch.cuda.is_available() else "cpu"
        args = Namespace(config='large')
        model = TwinLiteNetPlus(args)
        model = model.cuda()
        model_path = '/home/bylogix/TwinLiteNetPlus/pretrained/large.pth'
        model.load_state_dict(torch.load(model_path))
        model.eval()

    else:
        raise ValueError(f"Model type '{model_type_param}' not found")
    if half_param:
        model.half()
    return model


def get_segmentation(self, model, input_tensor, current_model_type, show=False): # Added current_model_type
    """Run segmentation on the input tensor and optionally display the segmentation mask."""
    with torch.no_grad():
        if current_model_type == 'hybridnets' or current_model_type == "local_hybridnets":
            _, _, cls, _, seg = model(input_tensor)
            
        elif current_model_type == "yolop":
            _, seg, _ = model(input_tensor)

        elif current_model_type =='twin':
            if input_tensor.dim() != 4:
                raise ValueError(f"Input tensor must be 4D [B,C,H,W]. Got {input_tensor.shape}")
            seg_road, seg_lane= model(input_tensor) 
            seg = seg_road  
        elif current_model_type == 'twinplus':
            seg_road, seg_lane= model(input_tensor) 
            seg = seg_road  
        else:
            raise ValueError(f"Model type '{current_model_type}' not found for segmentation step")
    if show:
        display = seg[0].cpu().numpy()
        # It's usually better to handle CV2 windows within the main thread or specific GUI thread
        # For ROS nodes, direct cv2.imshow can sometimes cause issues, consider publishing an Image msg instead.
        cv2.imshow("Segmentation Mask", display)
        cv2.waitKey(1)
    return seg


class SegNode(Node):
    """ROS2 Node for segmentation with periodic image processing and model loading."""

    def __init__(self):
        super().__init__('seg_node')
        self.bridge = CvBridge()

        # --- Parameter Handling ---
        self.declare_parameter('segmentation_mode', 'road_lane')  # Default value
        self.segmentation_mode = self.get_parameter('segmentation_mode').get_parameter_value().string_value
        self.get_logger().info(f"Segmentation node initialized with segmentation_mode: '{self.segmentation_mode}'")



        self.current_model_type = model_type
        self.model = initialize_model(self.current_model_type, half_param=half)
        self.bool_msg = Bool()
        self.bool_msg.data = True
        self.count = 0 # Instance variable to avoid conflict with global

        # Publishers
        self.seg_img_pub = self.create_publisher(Image, topic_names['segmented_image'], 10)
        self.model_enable_pub = self.create_publisher(Bool, topic_names["model_enable"], 10)

        # Subscribers
        self.create_subscription(Image, topic_names["RGB_image"], self.image_callback, 10)

        # Notify model initialization
        self.model_enable_pub.publish(self.bool_msg)
        self.get_logger().info(f"Segmentation Node Initialized (Model: {self.current_model_type}) and Model Enabled")

    def image_callback(self, data):
        """Callback to process images from the RGB image topic."""
        self.count += 1
        # Process every 9th frame (roughly 3-4 FPS if ZED is at 30 FPS)
        # Consider making this rate configurable via a parameter if needed
        if self.count % 9 != 0:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8") # Assuming RGB, if BGR use "bgr8"
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # Using global half here.
        if self.current_model_type == "hybridnets":
            input_tensor, dw, dh = preprocessing_image(cv_image, half=half)
        # YOLOPV2 and TwinLiteNets is not trained on std/mean normalization
        else:
            input_tensor ,dw ,dh = preprocessing_image_no_normalisation(cv_image, self.current_model_type,half=half)


        with torch.no_grad():
            start_time = time.time() # Renamed to avoid conflict
            # Pass the stored model type to get_segmentation
            seg = get_segmentation(self, self.model, input_tensor, self.current_model_type, show=SHOW) # SHOW is from ROS_utils
            processing_time = time.time() - start_time
            self.get_logger().info(f"Segmentation processing time: {processing_time:.4f}s")

        # --- Pass the segmentation_mode to preprocessing_mask ---
        mask = preprocessing_mask(seg,dw ,dh, orig_shape=cv_image.shape[:2], show=SHOW, improve=True)
        # --- End ---

        try:
            seg_img_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
            seg_img_msg.header = data.header # Propagate timestamp and frame_id
            self.seg_img_pub.publish(seg_img_msg)
            # self.get_logger().info("Segmented image published") # Can be a bit verbose
        except Exception as e:
            self.get_logger().error(f"Error publishing segmented image: {e}")


def main(args=None):
    rclpy.init(args=args)
    seg_node = SegNode()

    try:
        rclpy.spin(seg_node)
    except KeyboardInterrupt:
        seg_node.get_logger().info("Segmentation Node shutting down.")
    except Exception as e:
        seg_node.get_logger().error(f"Unhandled exception in SegNode: {e}")
    finally:
        if rclpy.ok():
            seg_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
