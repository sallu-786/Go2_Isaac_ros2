import time
import cv2
import base64
import rclpy
import numpy as np
import torch
from geometry_msgs.msg import Twist

from web_utils.camera import RosImageSubscriber
from web_utils.controller import Controller
from web_utils.llm_describer import GenerateResponse
# from web_utils.tf_frames import TFGraph


# ---Display Live Stream -----------------------------------------------------
class DataStream:
    def __init__(self, camera_topic, cmd_vel):
        self.system = GenerateResponse()
        # self.TF = TFGraph()

        self.image_subscriber = RosImageSubscriber(camera_topic)     #change topic names accordingly

        self.base_vel_cmd_input = torch.zeros((1, 3), dtype=torch.float32)
        self.controller = Controller(cmd_vel=self.base_vel_cmd_input)
        self.cmd_publisher = self.controller.create_publisher(Twist,cmd_vel, 10)
        self.description_cache = "説明待ち..."
        self.desc_update_interval = 5  # seconds
        self.sleep_interval=0.05
        self.image_width=680
        self.image_height=460
        self.frame=None
        self.stream=True



    def publish_commands(self):
            """Continuously publish base_vel_cmd_input to /cmd_vel"""
            rate_hz = 10
            rate = 1.0 / rate_hz
            while rclpy.ok():
                msg = Twist()
                msg.linear.x = float(self.base_vel_cmd_input[0, 0])
                msg.linear.y = float(self.base_vel_cmd_input[0, 1])
                msg.angular.z = float(self.base_vel_cmd_input[0, 2])
                self.cmd_publisher.publish(msg)
                time.sleep(rate)

    def get_description_from_image(self, image: np.ndarray):
        # Ensure RGB before encoding
        if image.shape[2] == 3:
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        else:
            image_rgb = image
        _, buffer = cv2.imencode('.jpg', image_rgb)
        img_b64 = base64.b64encode(buffer).decode('utf-8')
        data_url = f"data:image/jpeg;base64,{img_b64}"
        return self.system.llm_response(data_url)

    def update_description(self):
        while rclpy.ok():
            frame = self.image_subscriber.color_frame
            if frame is not None:
                # Convert to RGB for LLM description
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                resized_frame = cv2.resize(rgb_frame, (self.image_width, self.image_height))
                self.description_cache = self.get_description_from_image(resized_frame)
            time.sleep(self.desc_update_interval)


    def live_cam_feed(self,mode:str):
        
        while True:
            if mode=="color_image":
                self.frame = self.image_subscriber.color_frame

            elif mode=="yolo_detection":
                self.frame = self.image_subscriber.detection_frame
    
            
            if self.frame is None:
                time.sleep(self.sleep_interval)
                continue

            # Convert to RGB for Gradio display
            rgb_frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
            resized_frame = cv2.resize(rgb_frame, (self.image_width, self.image_height))
            yield resized_frame, self.description_cache
            time.sleep(self.sleep_interval)
