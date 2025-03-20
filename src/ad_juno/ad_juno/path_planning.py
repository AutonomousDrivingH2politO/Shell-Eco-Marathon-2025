#!/usr/bin/env python3.10
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import math
from datetime import datetime
#modify the directory of shared_objects
import os
import sys

from shared_objects.ROS_utils import Topics, SHOW

from shared_objects.utils_path import computing_lateral_distance, processing_mask
import cv2


# Initialize topics and node-related variables
topics = Topics()
topic_names = topics.topic_names

DEBUG = True
wheelbase = 1.6
speed = 15.0
gain = 0
cv_image = None

# Node class
class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning_node')

        self.steer_pub = self.create_publisher(Float64, topic_names['steering'], 10)
        self.req_speed_pub = self.create_publisher(Float64, topic_names['requested_speed'], 10)

        self.image_sub = self.create_subscription(Image, topic_names['segmented_image'], self.image_callback, 10)
        #self.original_image_sub = self.create_subscription(Image, topic_names[''], self.original_image_callback, 10)

        self.bridge = CvBridge()
        self.counter = 0

        if DEBUG:
            self.logs_folder, self.output_folder, self.frames_folder = self.set_debug_folders()

        # Initial speed
        req_speed_msg = Float64()
        req_speed_msg.data = speed
        self.req_speed_pub.publish(req_speed_msg)

    def set_debug_folders(self):
        debug_folder = os.path.join(os.getcwd(), "DEBUG")
        try:
            if not os.path.exists(debug_folder):
                os.makedirs(debug_folder)
        except OSError as e:
            self.get_logger().error(f"Error creating debug folder: {e}")

        now = datetime.now()
        folder = os.path.join(debug_folder, now.strftime("%Y_%m_%d_%H_%M_%S"))
        os.makedirs(folder)

        # Create subfolders for logs, output, and frames
        logs_folder = os.path.join(folder, "logs")
        output_folder = os.path.join(folder, "output")
        frames_folder = os.path.join(folder, "frames")
        os.makedirs(logs_folder)
        os.makedirs(output_folder)
        os.makedirs(frames_folder)

        return logs_folder, output_folder, frames_folder

    """def original_image_callback(self, data):
        # Store the RGB image
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "rgb8")
"""
    def image_callback(self, data):
        # Convert ROS Image message to OpenCV image
        mask = self.bridge.imgmsg_to_cv2(data, "mono8")
        line_edges = processing_mask(mask, self.cv_image)

        # Calculate distances and midpoint
        lateral_distance, longitudinal_distance, midpoints = computing_lateral_distance(line_edges, show=SHOW)

        # Calculate steering angle
        distance_to_waypoint = (longitudinal_distance + gain) ** 2 + lateral_distance ** 2
        degree_steering_angle = math.degrees(math.atan2(2 * wheelbase * lateral_distance, distance_to_waypoint))

        # Debug mode image display and logging
        if DEBUG:
            if midpoints is not None:
                posm = midpoints[-1]
                midpoints = midpoints[:-1]
                cv2.circle(line_edges, tuple(posm[::-1]), 2, (255, 255, 255), 5)
                for p in midpoints:
                    cv2.circle(line_edges, tuple(p[::-1]), 2, (200, 200, 200), 3)

            resized_image = cv2.resize(self.cv_image, (540, 360))
            resized_mask = cv2.resize(mask, (540, 360))
            resized_line_edges = cv2.resize(line_edges, (540, 360))
            concatenated_image = np.hstack((cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB),
                                            cv2.cvtColor(resized_mask, cv2.COLOR_GRAY2RGB),
                                            cv2.cvtColor(resized_line_edges, cv2.COLOR_GRAY2BGR)))

            # Save frame and output images
            frame_name = f"frame_{self.counter}.png"
            frame_path = os.path.join(self.frames_folder, frame_name)
            cv2.imwrite(frame_path, self.cv_image)

            output_name = f"output_{self.counter}.png"
            output_path = os.path.join(self.output_folder, output_name)
            cv2.imwrite(output_path, concatenated_image)

            # Log data to file
            log_file = os.path.join(self.logs_folder, f"log_{self.counter}.txt")
            with open(log_file, "w") as log:
                log.write(f"{self.counter}: - longitudinal_distance: {longitudinal_distance} - degree_steering_angle: {degree_steering_angle}\n")
            self.counter += 1

        # Display concatenated image if SHOW is enabled
        if SHOW:
            resized_image = cv2.resize(self.cv_image, (540, 360))
            resized_mask = cv2.resize(mask, (540, 360))
            resized_line_edges = cv2.resize(line_edges, (540, 360))
            concatenated_image = np.hstack((cv2.cvtColor(resized_image, cv2.COLOR_BGR2RGB),
                                            cv2.cvtColor(resized_mask, cv2.COLOR_GRAY2RGB),
                                            cv2.cvtColor(resized_line_edges, cv2.COLOR_GRAY2BGR)))
            cv2.imshow("Denedik :(", concatenated_image)
            cv2.waitKey(1)

        # Publish the steering angle
        steer_msg = Float64()
        steer_msg.data = degree_steering_angle
        self.steer_pub.publish(steer_msg)

        # Log information to the console
        self.get_logger().info(f"Longitudinal Distance: {longitudinal_distance}")
        self.get_logger().info(f"Steering Angle: {degree_steering_angle}")

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Path Planning Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
