import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from shared_objects.ROS_utils import Topics



class Throttle(Node):
    
    def __init__(self):
        super().__init__("throttle_node")
        self.topics = Topics()
        self.topic_names = self.topics.topic_names

        self.speed_threshold = 2
        self.stop=False
        self.model_enable=False
        self.engine_enable=True
        self.thr_msg=Float32()


        self.throttle_pub = self.create_publisher(Float32, self.topic_names["throttle"], 10)
        self.create_subscription(Float32, self.topic_names["speed"], self.callback_speed, 10)
        self.create_subscription(Float32, self.topic_names["requested_speed"], self.callback_requested_speed, 10)
        self.create_subscription(Bool, self.topic_names["stop"], self.callback_stop, 10)
        self.create_subscription(Bool, self.topic_names["model_enable"], self.callback_model_enable, 10)
        self.create_subscription(Bool, self.topic_names["engine_enable"], self.callback_engine_enable, 10)



    def callback_model_enable(self, data):
        self.model_enable=data.data

    def callback_engine_enable(self, data):
        self.engine_enable=data.data

    def callback_stop(self, data):
    #stop=True => stop => enable=False
    #stop=False => go => enable=True
        self.stop=data.data

    def callback_requested_speed(self, data):
        self.speed_threshold=data.data

    def callback_speed(self, value):
        self.speed=value.data

        if not self.stop and self.model_enable and self.engine_enable and self.speed <=self.speed_threshold:
            self.thr_msg.data=50.0
        else:
            self.thr_msg.data=0.0

        self.throttle_pub.publish(self.thr_msg)

def main(args = None):
    rclpy.init(args=args)
    node = Throttle()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
