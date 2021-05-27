#!/usr/bin/python3
import torch
import torchvision
import cv2
import numpy as np
import os
import rospy
import time
import torch.nn.functional as F
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from jetbot import ObjectDetector

width = 300
height = 300

def detection_center(detection):
    """Computes the center x, y coordinates of the object"""
    bbox = detection['bbox']
    center_x = (bbox[0] + bbox[2]) / 2.0 - 0.5
    center_y = (bbox[1] + bbox[3]) / 2.0 - 0.5
    return (center_x, center_y)
    
def norm(vec):
    """Computes the length of the 2D vector"""
    return np.sqrt(vec[0]**2 + vec[1]**2)

def closest_detection(detections):
    """Finds the detection closest to the image center"""
    closest_detection = None
    for det in detections:
        center = detection_center(det)
        if closest_detection is None:
            closest_detection = det
        elif norm(detection_center(det)) < norm(detection_center(closest_detection)):
            closest_detection = det
    return closest_detection
        
def execute(change):
    image = change['new']
    # execute collision model to determine if blocked
    movepub = rospy.Publisher('jetbot_motors/cmd_str', String, queue_size=10)
    image_pub = rospy.Publisher('jetbot_camera/detection', Image, queue_size=1)
    bridge = CvBridge()
    # compute all detected objects
    detections = model(image)
    
    # draw all detections on image
    for det in detections[0]:
        bbox = det['bbox']
        cv2.rectangle(image, (int(width * bbox[0]), int(height * bbox[1])), (int(width * bbox[2]), int(height * bbox[3])), (255, 0, 0), 2)
    
    # select detections that match selected class label
    matching_detections = [d for d in detections[0] if d['label'] == 1]
    
    # get detection closest to center of field of view and draw it
    det = closest_detection(matching_detections)
    if det is not None:
        bbox = det['bbox']
        cv2.rectangle(image, (int(width * bbox[0]), int(height * bbox[1])), (int(width * bbox[2]), int(height * bbox[3])), (0, 255, 0), 5)
    
    pubimage = bridge.cv2_to_imgmsg(image,encoding = "passthrough")
    image_pub.publish(pubimage)
        
    # otherwise go forward if no target detected
    if det is None:
        movepub.publish(String("backward"))
        
    # otherwsie steer towards target
    else:
        # move robot forward and steer proportional target's x-distance from center
        center = detection_center(det)
        movepub.publish(String("backward"))
    image = bridge.cv2_to_imgmsg(image,encoding = "passthrough")
    image_pub.publish(image)

def callback(data):
    #print(data)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data ,desired_encoding='passthrough')
    execute({'new': cv_image})
    
   # initialization
if __name__ == '__main__':
    # setup ros node
    rospy.init_node('object_following')
    model = ObjectDetector('ssd_mobilenet_v2_coco.engine')

    image_sub = rospy.Subscriber("jetbot_camera/raw",Image,callback)
    # start running
    rospy.spin()
    