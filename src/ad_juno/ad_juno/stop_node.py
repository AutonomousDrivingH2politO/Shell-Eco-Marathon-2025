#!/usr/bin/env python3
import sys
import os
import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import time  # for sleep function

shared_objects_path = '~/ROS2_WS/src/shared_objects/src/'
if shared_objects_path not in sys.path:
    sys.path.insert(0, shared_objects_path)

os.environ['PYTHONPATH'] = f"{os.environ.get('PYTHONPATH', '')}:{shared_objects_path}"

from shared_objects.ROS_utils import Topics, SHOW
from shared_objects.utils_stop import analysis
from shared_objects.Motor import Stepper  # Import Stepper for brake functionality


class StopSignDetector(Node):
    def __init__(self):
        super().__init__('stop_node', parameter_overrides=[])
        
        self.enable = True
        self.count_stop = 0
        self.count_img = 0
        self.threshold_stop = 2
        self.threshold_img = 60
        self.bridge = CvBridge()
        self.is_waiting = False   #babossss

        topics = Topics()
        self.topic_names = topics.topic_names
        
        # Initialize Stepper for brake
        self.brake = Stepper(
            interface="can",
            data_rate=1000000,
            module_id=3,  # modulo dello sterzo
            max_velocity=80_000,
            max_acc=50_000,
            MaxDeceleration=50_000,  # parametro di esempio: regola se necessario
            V1=100_000,
            A1=50_000,             # parametro di esempio: regola se necessario
            D1=50_000    
        )

        # Publishers and subscribers
        self.stop_pub = self.create_publisher(Bool, self.topic_names["stop"], 1)
        self.enable_pub = self.create_publisher(Bool, self.topic_names["stop_enable"], 1)
        self.original_sub = self.create_subscription(Image, self.topic_names["RGB_image"], self.original_img_callback, 1)
        self.enable_sub = self.create_subscription(Bool, self.topic_names["stop_enable"], self.enable_callback, 1)
        
        self.timer = None  #babossss
        
        # Logging
        self.get_logger().info("StopSignDetector node has been started")

    def enable_callback(self, msg):
        self.enable = msg.data

    def original_img_callback(self, msg):
        if self.enable and not self.is_waiting:
            self.count_img += 1
            if self.count_img % self.threshold_img != 0:
                return
            
            img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            result, img = analysis(img)
            
            if result:
                if self.count_stop < self.threshold_stop:
                    self.count_stop += 1
                else:
                    # Trigger brake logic
                    self.get_logger().info("Stop sign detected. Activating brake.")
                    self.brake.brake()
                    self.get_logger().info("Brake activated. Waiting...")
                    
                    # Publish stop signal and disable further stop detection temporarily
                    self.is_waiting = True
                    bool_msg.data = False
                    self.stop_pub.publish(bool_msg)
                    
                    # Wait for some time before re-enabling detection
                    time.sleep(10)
                    
                    bool_msg.data = True
                    self.enable_pub.publish(bool_msg)
                    self.get_logger().info("Braking process completed.")
                    
                    # Reset counters
                    self.enable = False
                    self.timer = self.create_timer(10.0, self.end_waiting)
            else:
                self.count_stop = 0

            if SHOW:
                cv2.imshow('Cropped Image', img)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
    def end_waiting(self):
        # Re-enable detection after waiting
        bool_msg = Bool(data=True)
        self.enable_pub.publish(bool_msg)
        self.is_waiting = False
        self.enable = True
        self.count_stop = 0
        self.get_logger().info("Braking process completed. Resuming detection.")
        # Destroy the timer
        self.timer.cancel()
        self.timer = None

def main():
    rclpy.init(args=sys.argv)
    node = StopSignDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        node.get_logger().error('Error: %r' % (e))
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
