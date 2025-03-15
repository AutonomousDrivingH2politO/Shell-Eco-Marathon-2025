#!/usr/bin/python3

import rclpy
import time
import math
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3
import pyzed.sl as sl
from cv_bridge import CvBridge
import os

class ZEDCameraNode(Node):

    class TimestampHandler:
        def __init__(self):
            self.t_imu = sl.Timestamp()
            self.t_baro = sl.Timestamp()
            self.t_mag = sl.Timestamp()

        # check if the new timestamp is higher than the reference one, and if yes, save the current as reference
        def is_new(self, sensor):
            if (isinstance(sensor, sl.IMUData)):
                new_ = (sensor.timestamp.get_microseconds() > self.t_imu.get_microseconds())
                if new_:
                    self.t_imu = sensor.timestamp
                return new_
            elif (isinstance(sensor, sl.MagnetometerData)):
                new_ = (sensor.timestamp.get_microseconds() > self.t_mag.get_microseconds())
                if new_:
                    self.t_mag = sensor.timestamp
                return new_
            elif (isinstance(sensor, sl.BarometerData)):
                new_ = (sensor.timestamp.get_microseconds() > self.t_baro.get_microseconds())
                if new_:
                    self.t_baro = sensor.timestamp
                return new_

    def __init__(self):
        super().__init__('zed_camera_node')

        # Declaring the parameters for Debug mode
        self.declare_parameter('debug_mode', False) # False by default
        self.debug_mode = self.get_parameter('debug_mode').value # If recieves the debug_mode:=true takes the boolean True

        # Initialize ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD2K  # Use HD720 or HD1200 video mode
        init_params.camera_fps = 15 # Set fps at 15
        self.sensor_data = sl.SensorsData()
        self.sensor_params = self.zed.get_camera_information().sensors_configuration
        self.ts_handler = self.TimestampHandler()
        
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Failed to open the ZED camera: {repr(err)}")
            rclpy.shutdown(context=rclpy.get_default_context())
            return

        # Initialize CvBridge to convert OpenCV images to ROS Image messages
        self.bridge = CvBridge()

        # Create a publisher to publish images
        self.image_publ = self.create_publisher(Image, 'left_image', 1)
        self.image_pubr = self.create_publisher(Image, 'right_image', 1)
        self.image_publisher = self.create_publisher(Image, "zed/zed_node/RGB_IMAGE", 1)
        self.imu_pub = self.create_publisher(Imu,'imu/data', 1)

        # Set up a timer to grab images periodically 
        self.timer_image = self.create_timer(1.0/15, self.timer_callback_image)
        self.timer_imu = self.create_timer(1.0/15,self.timer_callback_imu)
        
        # Create folder for debug images
        if self.debug_mode:
            try:
                os.makedirs("debug_images", exist_ok=True)
                self.get_logger().info("-------Debug mode enabled zed_node will save the images-------")
            except:
                self.get_logger().warning("Unable to create debug folder!")
    def printSensorParameters(self,sensor_parameters):
        if sensor_parameters.is_available:
            self.get_logger().info("*****************************")
            self.get_logger().info("Sensor type: " + str(sensor_parameters.sensor_type))
            self.get_logger().info("Max rate: "  + str(sensor_parameters.sampling_rate) + " "  + str(sl.SENSORS_UNIT.HERTZ))
            self.get_logger().info("Range: "  + str(sensor_parameters.sensor_range) + " "  + str(sensor_parameters.sensor_unit))
            self.get_logger().info("Resolution: " + str(sensor_parameters.resolution) + " "  + str(sensor_parameters.sensor_unit))
            if not math.isnan(sensor_parameters.noise_density):
                self.get_logger().debug("Noise Density: "  + str(sensor_parameters.noise_density) + " " + str(sensor_parameters.sensor_unit) + "/√Hz")
            if not math.isnan(sensor_parameters.random_walk):
                self.get_logger().debug("Random Walk: "  + str(sensor_parameters.random_walk) + " " + str(sensor_parameters.sensor_unit) + "/s/√Hz")
    
    def timer_callback_imu(self):
        
        self.printSensorParameters(self.sensor_params.accelerometer_parameters)
        if self.zed.get_sensors_data(self.sensor_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
            if self.ts_handler.is_new(self.sensor_data.get_imu_data()):
                orientation = self.sensor_data.get_imu_data().get_pose().get_orientation().get()
                orientation_msg = Quaternion(
                    x=orientation[0],
                    y=orientation[1],
                    z=orientation[2],
                    w=orientation[3]
                )

                # Retrieve linear acceleration
                linear_acceleration = self.sensor_data.get_imu_data().get_linear_acceleration()
                linear_acceleration_msg = Vector3(
                    x=linear_acceleration[0],
                    y=linear_acceleration[1],
                    z=linear_acceleration[2]
                )

                # Retrieve angular velocity
                angular_velocity = self.sensor_data.get_imu_data().get_angular_velocity()
                angular_velocity_msg = Vector3(
                    x=angular_velocity[0],
                    y=angular_velocity[1],
                    z=angular_velocity[2]
                )

                # Create and populate the Imu message
                imu_msg = Imu()
                imu_msg.header = Header()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = 'imu_link'  # Set this to the appropriate frame

                imu_msg.orientation = orientation_msg
                imu_msg.linear_acceleration = linear_acceleration_msg
                imu_msg.angular_velocity = angular_velocity_msg

                # Optionally, set covariance matrices if known; otherwise, leave as defaults
                # imu_msg.orientation_covariance = [0.0] * 9
                # imu_msg.angular_velocity_covariance = [0.0] * 9
                # imu_msg.linear_acceleration_covariance = [0.0] * 9

                # Publish the IMU message
                self.imu_pub.publish(imu_msg)

                # Log the IMU data
                self.get_logger().info("Published IMU data:")
                self.get_logger().info(f"Orientation  x: {orientation_msg.x}, y: {orientation_msg.y}, z: {orientation_msg.z}, w: {orientation_msg.w}")
                self.get_logger().info(f"Linear Acceleration  x: {linear_acceleration_msg.x} m/s², y: {linear_acceleration_msg.y} m/s², z: {linear_acceleration_msg.z} m/s²")
                self.get_logger().info(f"Angular Velocity  x: {angular_velocity_msg.x} rad/s, y: {angular_velocity_msg.y} rad/s, z: {angular_velocity_msg.z} rad/s")

    def timer_callback_image(self):
        left_image = sl.Mat()
        right_image = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()

        # Grab an image
        if self.zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(left_image, sl.VIEW.LEFT)
            self.zed.retrieve_image(right_image, sl.VIEW.RIGHT)

            # Convert ZED image to OpenCV format (BGRA)
            opencv_image_left = cv2.cvtColor(left_image.get_data(), cv2.COLOR_BGRA2BGR)
            opencv_image_right = cv2.cvtColor(right_image.get_data(),cv2.COLOR_BGRA2BGR)

            # Convert OpenCV image to ROS Image message
            ros_image_left = self.bridge.cv2_to_imgmsg(opencv_image_left, encoding="8uc4")
            ros_image_right = self.bridge.cv2_to_imgmsg(opencv_image_right, encoding="8uc4")

            # Add timestamp to the message header
            ros_image_left.header = Header()
            ros_image_right.header = Header()
            ros_image_left.header.stamp = self.get_clock().now().to_msg()
            ros_image_right.header.stamp = self.get_clock().now().to_msg()

            # Publish image
            self.image_publ.publish(ros_image_left)
            self.image_pubr.publish(ros_image_right)
            self.get_logger().info(f"Publishing image: {left_image.get_width()} x {left_image.get_height()}")
            self.get_logger().info(f"Publishing image: {right_image.get_width()} x {right_image.get_height()}")

            if self.debug_mode:
                left_im_path = f"debug_images/left_{ros_image_left.header.stamp}.png"
                right_im_path = f"debug_images/right_{ros_image_right.header.stamp}.png"
                
                cv2.imwrite(left_im_path, ros_image_left)
                cv2.imwrite(right_im_path, ros_image_right)

                self.get_logger().info(f"Saved left image to {left_im_path}")
                self.get_logger().info(f"Saved right image to {right_im_path}")



    def close_camera(self):
        self.zed.close()
        super().destroy_node()

def main(args=None): # No argument if not received
    # To run in debug mode run using $ ros2 run ad_juno ros_node --ros-args -p debug_mode:=true
    # I used ROS parameters instead of sys.argv for better ROS framework implementations
    rclpy.init(args=args, context=rclpy.get_default_context())

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

