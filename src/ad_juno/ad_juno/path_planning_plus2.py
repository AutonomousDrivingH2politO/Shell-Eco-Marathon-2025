#!/usr/bin/env python3

import os
import math
import numpy as np
import cv2
from datetime import datetime
from rclpy.node import Node
from rclpy import init, spin
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
from shared_objects.utils_path import computing_lateral_distance, processing_mask
from shared_objects.ROS_utils import Topics, SHOW

#giriyor pathplanninge
class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning')
        # Parameters
        self.debug = self.declare_parameter('debug', True).value
        self.simulation = self.declare_parameter('simulation', False).value
        self.wheelbase = self.declare_parameter('wheelbase', 1.6).value
        self.gain = self.declare_parameter('gain', 0.0).value
        self.bridge = CvBridge()
        self.topics = Topics()
        self.topic_names = self.topics.topic_names

        self.counter = 0
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, self.topic_names["goal"], 10)

        # Subscribers
        self.create_subscription(Image, self.topic_names["segmented_image"], self.image_callback, 10)

        self.create_subscription(Image, self.topic_names["RGB_image"], self.original_image_callback, 10)
        # Debug folders
        if self.debug:
            self.logs_folder, self.output_folder, self.frames_folder = self.set_debug_folders()

        self.get_logger().info("Path Planning Node Initialized")

#set debug folders giriyor
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

    def original_image_callback(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")

 #image_callback girmiyor
    def image_callback(self, data):
        self.get_logger().info("hello bitch1767857 \n")
        mask = self.bridge.imgmsg_to_cv2(data, "mono8")
        line_edges = processing_mask(mask, self.cv_image, show=False)

        lateral_distance, longitudinal_distance, curvature, midpoints = computing_lateral_distance(line_edges, show=False)
        print("hello bitch2 \n")
        if SHOW and midpoints is not None:
            posm = midpoints[-1]
            midpoints = midpoints[:-1]
            cv2.circle(line_edges, tuple(posm[::-1]), 2, (255, 255, 255), 5)
            for p in midpoints:
                cv2.circle(line_edges, tuple(p[::-1]), 2, (200, 200, 200), 3)
                print("hello bitch3 \n")
        if lateral_distance == -np.inf:
            degree_steering_angle = -40.0
        elif lateral_distance == np.inf:
            degree_steering_angle = 40.0
        else:
            distance_to_waypoint = (longitudinal_distance + self.gain) ** 2 + lateral_distance ** 2
            degree_steering_angle = math.degrees(math.atan2(2 * self.wheelbase * lateral_distance, distance_to_waypoint))

        if self.debug:
            resized_image = cv2.resize(self.cv_image, (540, 360))
            resized_mask = cv2.resize(mask, (540, 360))
            resized_line_edges = cv2.resize(line_edges, (540, 360))
            concatenated_image = np.hstack((
                cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB),
                cv2.cvtColor(resized_mask, cv2.COLOR_GRAY2RGB),
                cv2.cvtColor(resized_line_edges, cv2.COLOR_GRAY2BGR)
            ))

            frame_name = f"frame_{self.counter}.png"
            frame_path = os.path.join(self.frames_folder, frame_name)
            cv2.imwrite(frame_path, self.cv_image)

            output_name = f"output_{self.counter}.png"
            output_path = os.path.join(self.output_folder, output_name)
            cv2.imwrite(output_path, concatenated_image)

            log_file = os.path.join(self.logs_folder, f"log_{self.counter}.txt")
            with open(log_file, "w") as log:
                log.write(f"{self.counter}: Curvature: {curvature} - Longitudinal Distance: {longitudinal_distance} - Degree Steering Angle: {degree_steering_angle}\n")
            self.counter += 1

        # Publish goal message
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "camera_link"
        goal_msg.pose.position.x = longitudinal_distance
        goal_msg.pose.position.y = -lateral_distance
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(goal_msg)

def main(args=None):
    init(args=args)
    node = PathPlanningNode()
    spin(node)

if __name__ == "__main__":
    main()

