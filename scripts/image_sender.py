#!/usr/bin/env python
import cv2
import os
import rospy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from PIL import Image as PILImage


def image_sender():

    rospy.init_node('image_sender', anonymous=True)
    pub = rospy.Publisher('/image_topic', Image, queue_size=10)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        # Create an Image message
        image = Image()

        # Modify the image path to your own
        demo_image_path = os.path.join(os.path.dirname(__file__),"..", "sad.png")
        assert os.path.exists(demo_image_path), "Please modify the image path to your own."

        image_read = PILImage.open(demo_image_path)
        image_read = image_read.resize((320, 240))
        image_read_bgr = cv2.cvtColor(np.array(image_read), cv2.COLOR_RGB2BGR)
        image_read_np = np.array(image_read_bgr)
        # publish the image
        image = CvBridge().cv2_to_imgmsg(image_read_np, encoding="passthrough")
        pub.publish(image)
        print("Image published")
        break
        # rate.sleep()

if __name__ == '__main__':
    image_sender()