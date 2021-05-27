#!/usr/bin/python3
import cv2
import numpy as np
from jetbot import Camera
import os
import rospy
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def image_pub():
    image_pub = rospy.Publisher('jetbot_camera/raw',Image,queue_size=1)
    bridge = CvBridge()
    camera = Camera.instance(width=300, height=300)
    while not rospy.is_shutdown():
        image = bridge.cv2_to_imgmsg(camera.value, encoding = "passthrough")
        image_pub.publish(image)


# initialization
if __name__ == '__main__':
    # setup ros node
    rospy.init_node('jetbot_cam')

    try:
        image_pub()
        reospy.spin()
    except:
        print("Shutting down")