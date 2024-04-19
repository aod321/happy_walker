#!/usr/bin/env python
import os
import cv2
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

API_KEY = os.getenv("OPENAI_API_KEY")
if API_KEY is None:
    raise ValueError("Please set the environment variable OPENAI_API_KEY.")

# ---- GPT4v API Module---
# Modify from https://github.com/OpenAdaptAI/OpenAdapt/blob/44d4a55f332ef3b846933b38854372d4166fd2ea/experiments/gpt4v.py
import base64
import json
import numpy as np
import requests
from io import BytesIO
from PIL import Image as PILImage


def encode_image_np(image_np: np.ndarray):
    """Encodes a numpy array image to base64 and determines the correct MIME type."""
    im = PILImage.fromarray(image_np)
    buffered = BytesIO()
    im.save(buffered, format="PNG")
    encoded_string = base64.b64encode(buffered.getvalue()).decode('utf-8')
    return f"data:image/png;base64,{encoded_string}"


def create_payload_np(image_np: np.ndarray, model="gpt-4-1106-vision-preview", max_tokens=100, detail="high"):
    """Creates the payload for the API request using a numpy array image."""
    base64_image = encode_image_np(image_np)

    messages = [
        {
            "role": "user",
            "content": [
                {
                    "type": "image_url",
                    "image_url": {
                        "url": base64_image,
                        "detail": detail,
                    }
                },
                 {
                    "type": "text",
                    "text": 'Is this person happy or sad? OUTPUT JSON FORMAT: {"emotion": "happy"} or {"emotion": "sad"}'
                },
            ],
        },
    ]

    return {
        "model": model,
        "messages": messages,
        "max_tokens": max_tokens,
        "temperature": 0.0,
    }


def query_openai(payload, api_key):
    """Sends a request to the OpenAI API and returns the response."""
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {api_key}"
    }
    response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)
    return response.json()


def gpt4_vision_query(image_np: np.ndarray, api_key: str):
    """Function to query GPT-4-V model with a numpy image and a prompt."""
    payload = create_payload_np(image_np)
    response = query_openai(payload, api_key=api_key)
    response_text = response['choices'][0]['message']['content']
    response_json = json.loads(response_text)
    return response_json
# ---- GPT4v---

class Talker:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/image_topic', Image, self.callback)
        rospy.init_node('talker', anonymous=True)
        self.rate = rospy.Rate(30) # 30hz
        self.cv_bridge = CvBridge()
    
    def callback(self, image):
        print("Received image!")
        # Do something with the image
        image_np = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
        image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        emotion = self.image_cognition(image_np)['emotion']
        print(f"Detected emotion: {emotion}")
        if emotion == "happy":
            print("Moving robot forward...")
            self.move_robot_forward()
        elif emotion == "sad":
            print("Moving robot backward...")
            self.move_robot_backward()
        
    def move_robot_forward(self, duration=1):
        # Control the robot forward
        twist = Twist()
        twist.linear.x = 0.5
        self.pub.publish(twist)
        # Stop the robot after duration seconds
        rospy.sleep(duration)
        twist.linear.x = 0
        self.pub.publish(twist)
    
    def move_robot_backward(self, duration=1):
        # Control the robot backward
        twist = Twist()
        twist.linear.x = -0.5
        self.pub.publish(twist)
        # Stop the robot after duration seconds
        rospy.sleep(duration)
        twist.linear.x = 0
        self.pub.publish(twist)
    
    def image_cognition(self, image):
        response = gpt4_vision_query(image, api_key=API_KEY)
        return response
    
    def start(self):
        print("Waiting for image...")
        while not rospy.is_shutdown():
            # Rate limiting
            self.rate.sleep()
           
if __name__ == '__main__':
    try:
        talker = Talker()
        talker.start()
    except rospy.ROSInterruptException:
        pass