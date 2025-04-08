#!/usr/bin/env python3
import sys
import os
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64, Float32
from sensor_msgs.msg import Image
from tf2_ros import TransformListener, Buffer
#from tf_transformations import euler_from_quaternion
from cv_bridge import CvBridge
from datetime import datetime
from shared_objects.utils_path import computing_lateral_distance, processing_mask
from shared_objects.ROS_utils import Topics, SHOW

def euler_from_quat(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class PathPlanningNode(Node):
    
    
    
    
    def __init__(self):
        super().__init__('path_planning')

        # Initialize variables
        self.DEBUG = False
        self.SIMULATION = False
        self.LANE_METERS = 9.8
        self.Y_METERS = {10.0: 550, 7.5: 605}
        self.LANE_PIXELS = None
        self.LATERAL_DISTANCE = 0
        self.scale_factor = None
        self.black_regions = None
        self.y_black = None
        self.prev_curvature = None
        self.costmap = None
        self.goal_published = False
        self.cv_image = None
        self.speed = 6
        self.wheelbase = 1.6
        self.gain = 0.0
        self.counter = 0

        # Publishers
        topics = Topics()
        self.topic_names = topics.topic_names
        self.steer_pub = self.create_publisher(Float64, self.topic_names["steering"], 1)
        self.req_speed_pub = self.create_publisher(Float32, self.topic_names["requested_speed"], 1)
        self.goal_pub = self.create_publisher(PoseStamped, self.topic_names["goal"], 1)
    
        
        # Subscribers
        self.create_subscription(Image, self.topic_names["segmented_image"], self.image_callback, 1)
        self.create_subscription(Image, self.topic_names["RGB_image"], self.original_image_callback, 1)
        self.create_subscription(OccupancyGrid, self.topic_names["costmap"], self.costmap_callback, 1)

        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initialize speed
        float_msg = Float32()
        float_msg.data = float(self.speed)
        self.req_speed_pub.publish(float_msg)

        # Set debug folders if debugging
        if self.DEBUG:
            self.logs_folder, self.output_folder, self.frames_folder = self.set_debug_folders()

        # Bridge for image conversion
        self.bridge = CvBridge()

    def set_debug_folders(self):
    
        debug_folder = os.path.join(os.getcwd(), "DEBUG")
        os.makedirs(debug_folder, exist_ok=True)

        now = datetime.now()
        folder = os.path.join(debug_folder, now.strftime("%Y_%m_%d_%H_%M_%S"))
        os.makedirs(folder, exist_ok=True)


        logs_folder = os.path.join(folder, "logs")
        output_folder = os.path.join(folder, "output")
        frames_folder = os.path.join(folder, "frames")
        os.makedirs(logs_folder, exist_ok=True)
        os.makedirs(output_folder, exist_ok=True)
        os.makedirs(frames_folder, exist_ok=True)

        return logs_folder, output_folder, frames_folder

    def costmap_callback(self, data):

        width, height = data.info.width, data.info.height
        costmap_2d = np.array(data.data).reshape((height, width))
        costmap_cv = ((costmap_2d + 1) / 101.0 * 255).astype(np.uint8)

        _, thresholded_costmap = cv2.threshold(costmap_cv, 127, 255, cv2.THRESH_BINARY)

        try:
            trans = self.tf_buffer.lookup_transform('camera_link', data.header.frame_id, rclpy.time.Time())
            rot = trans.transform.rotation
            roll, pitch, yaw = euler_from_quat([rot.x, rot.y, rot.z, rot.w])
        except Exception as e:
            self.get_logger().warn(f"Failed to get transform: {e}")
            return

        rotation_matrix = cv2.getRotationMatrix2D((width / 2, height / 2), np.degrees(-yaw), 1)
        rotated = cv2.warpAffine(thresholded_costmap, rotation_matrix, (width, height))
        rotated_costmap = cv2.rotate(rotated, cv2.ROTATE_90_COUNTERCLOCKWISE)
        self.costmap = cv2.flip(rotated_costmap, 1)


    def original_image_callback(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")

    def image_callback(self, data):
        mask = self.bridge.imgmsg_to_cv2(data, "mono8")
        line_edges = processing_mask(mask, self.cv_image, show=SHOW)

        lateral_distance, longitudinal_distance, curvature, midpoints = computing_lateral_distance(line_edges, show=SHOW)

        if lateral_distance == -np.inf:
            degree_steering_angle = -40.0
        elif lateral_distance == np.inf:
            degree_steering_angle = 40.0
        else:
            distance_to_waypoint = (longitudinal_distance + self.gain) ** 2 + lateral_distance ** 2
            degree_steering_angle = math.degrees(math.atan2(2 * self.wheelbase * lateral_distance, distance_to_waypoint))

        steer_msg = Float64()
        steer_msg.data = degree_steering_angle
        self.steer_pub.publish(steer_msg)

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "camera_link"
        goal_msg.pose.position.x = longitudinal_distance
        goal_msg.pose.position.y = -lateral_distance
        goal_msg.pose.orientation.w = 1.0

        if np.any(self.costmap):
            self.goal_pub.publish(goal_msg)
            self.goal_published = True
        elif self.goal_published:
            goal_msg.pose.position.x = 0.0
            goal_msg.pose.position.y = 0.0
            self.goal_pub.publish(goal_msg)
            self.goal_published = False

    def main_loop(self):
        rclpy.spin(self)

def main():
    rclpy.init()
    node = PathPlanningNode()
    node.main_loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
