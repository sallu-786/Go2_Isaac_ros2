from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class RosImageSubscriber(Node,):
    def __init__(self,topic_name:str):
        super().__init__('image_subscriber')

        self.color_subscription = self.create_subscription(
            Image,                          
            f'{topic_name}',  
            self.color_image_callback,
            10
        )
        self.detection_subscription = self.create_subscription(
            Image,

            f'{topic_name}', 

            self.detection_image_callback,
            10
        )

        self.bridge = CvBridge()
        self.color_frame = None
        self.detection_frame = None

    def color_image_callback(self, msg):
        self.color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def detection_image_callback(self, msg):
        self.detection_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')