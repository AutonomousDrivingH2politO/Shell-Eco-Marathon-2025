#!/usr/bin/env python3
import sys
import os
import math
import time
import torch
import numpy as np
import cv2
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64
from tf2_ros import TransformListener, Buffer
#from tf_transformations import euler_from_quaternion
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from datetime import datetime
from shared_objects.utils_path import computing_lateral_distance, processing_mask
from shared_objects.ROS_utils import Topics, SHOW

# Initialize ROS 2
rclpy.init()

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

# Create a ROS2 Node
class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning')
        
        self.bridge = CvBridge()
        self.counter = 0
        self.DEBUG = False
        self.SIMULATION = False
        self.LANE_METERS = 9.8
        self.LANE_PIXELS = None
        self.LATERAL_DISTANCE = 0
        self.scale_factor = None
        self.black_regions = None
        self.y_black = None
        self.prev_curvature = None
        self.costmap = None
        self.goal_published = False
        self.cv_image = None
        self.speed = 10
        self.wheelbase = 1.6  # Wheelbase of the vehicle
        self.gain = 0.0  # Steering gain
        
        # Topic names from shared Topics class
        topics = Topics()
        self.topic_names = topics.topic_names

        # Publishers
        self.steer_pub = self.create_publisher(Float64, self.topic_names["steering"], 1)
        self.req_speed_pub = self.create_publisher(Float64, self.topic_names["requested_speed"], 1)
        self.goal_pub = self.create_publisher(PoseStamped, self.topic_names["goal"], 1)
        
        # Subscribers
        self.create_subscription(Image, self.topic_names["segmented_image"], self.image_callback, 1)
        self.create_subscription(Image, self.topic_names["RGB_image"], self.original_image_callback, 1)
        
        # Costmap listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Initial speed message
        req_speed_msg = Float64()
        req_speed_msg.data = float(self.speed)
        self.req_speed_pub.publish(req_speed_msg)
        
        self.set_debug_folders()

    def costmap_callback(self, data):
        width, height = data.info.width, data.info.height
        costmap_2d = np.array(data.data).reshape((height, width))
        costmap_cv = ((costmap_2d + 1) / 101.0 * 255).astype(np.uint8)

        # Apply thresholding
        _, thresholded_costmap = cv2.threshold(costmap_cv, 127, 255, cv2.THRESH_BINARY)

        # Get the transformation
        try:
            trans, rot = self.tf_buffer.lookup_transform('camera_link', data.header.frame_id, rclpy.time.Time())
            roll, pitch, yaw = euler_from_quat(rot)
        except Exception as e:
            self.get_logger().warn(f"Transform error: {e}")
            return

        # Apply rotation
        rotation_matrix = cv2.getRotationMatrix2D((width / 2, height / 2), np.degrees(-yaw), 1)
        rotated = cv2.warpAffine(thresholded_costmap, rotation_matrix, (width, height))

        # Apply second rotation
        rotated_costmap = cv2.rotate(rotated, cv2.ROTATE_90_COUNTERCLOCKWISE)

        # Horizontally flip and crop
        flipped_costmap = cv2.flip(rotated_costmap, 1)
        cropped_costmap = flipped_costmap[:width // 2, :]

        # Resize
        resized_costmap = cv2.resize(cropped_costmap, (1280, 720))
        self.costmap = resized_costmap

    def set_debug_folders(self):
        if self.DEBUG:
            debug_folder = os.path.join(os.getcwd(), "DEBUG")
            if not os.path.exists(debug_folder):
                os.makedirs(debug_folder)

            now = datetime.now()
            folder = os.path.join(debug_folder, now.strftime("%Y_%m_%d_%H_%M_%S"))
            os.makedirs(folder)

            logs_folder = os.path.join(folder, "logs")
            output_folder = os.path.join(folder, "output")
            frames_folder = os.path.join(folder, "frames")
            os.makedirs(logs_folder)
            os.makedirs(output_folder)
            os.makedirs(frames_folder)

            return logs_folder, output_folder, frames_folder

    def original_image_callback(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")

    def image_callback(self, data):
        mask = self.bridge.imgmsg_to_cv2(data, "mono8")
        line_edges = processing_mask(mask, self.cv_image, show=False)

        lateral_distance, longitudinal_distance, curvature, midpoints = computing_lateral_distance(line_edges, show=False)

        if midpoints is not None:
            posm = midpoints[-1]
            midpoints = midpoints[:-1]
            cv2.circle(line_edges, tuple(posm[::-1]), 2, (255, 255, 255), 5)
            for p in midpoints:
                cv2.circle(line_edges, tuple(p[::-1]), 2, (200, 200, 200), 3)

        if lateral_distance == -np.inf:
            degree_steering_angle = -40.0
        elif lateral_distance == np.inf:
            degree_steering_angle = 40.0
        else:
            distance_to_waypoint = (longitudinal_distance + self.gain) ** 2 + lateral_distance ** 2
            degree_steering_angle = math.degrees(math.atan2(2 * self.wheelbase * lateral_distance, distance_to_waypoint))

        if self.DEBUG:
            self.save_debug_images(line_edges, degree_steering_angle)

        if SHOW:
            self.show_images(line_edges)

        self.steer_pub.publish(Float64(data=degree_steering_angle))

    def save_debug_images(self, line_edges, degree_steering_angle):
        resized_image = cv2.resize(self.cv_image, (540, 360))
        resized_mask = cv2.resize(self.bridge.imgmsg_to_cv2(data, "mono8"), (540, 360))
        resized_line_edges = cv2.resize(line_edges, (540, 360))
        concatenated_image = np.hstack((cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB), cv2.cvtColor(resized_mask, cv2.COLOR_GRAY2RGB), cv2.cvtColor(resized_line_edges, cv2.COLOR_GRAY2BGR)))

        frame_name = f"frame_{self.counter}.png"
        frame_path = os.path.join(self.frames_folder, frame_name)
        cv2.imwrite(frame_path, self.cv_image)

        output_name = f"output_{self.counter}.png"
        output_path = os.path.join(self.output_folder, output_name)
        cv2.imwrite(output_path, concatenated_image)

        log_file = os.path.join(self.logs_folder, f"log_{self.counter}.txt")
        with open(log_file, "w") as log:
            log.write(f"{self.counter}: Curvature: {curvature} - longitudinal_distance: {longitudinal_distance} - degree_steering_angle: {degree_steering_angle}\n")

        self.counter += 1

    def show_images(self, line_edges):
        resized_image = cv2.resize(self.cv_image, (540, 360))
        resized_mask = cv2.resize(self.bridge.imgmsg_to_cv2(data, "mono8"), (540, 360))
        resized_line_edges = cv2.resize(line_edges, (540, 360))
        concatenated_image = np.hstack((cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB), cv2.cvtColor(resized_mask, cv2.COLOR_GRAY2RGB), cv2.cvtColor(resized_line_edges, cv2.COLOR_GRAY2BGR)))
        cv2.imshow("Ci abbiamo provato", concatenated_image)
        cv2.waitKey(1)

def main():
    # Create ROS2 node
    path_planning_node = PathPlanningNode()

    # Spin the node
    rclpy.spin(path_planning_node)

    # Clean up and shutdown ROS
    path_planning_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

