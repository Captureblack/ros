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

#-----------------------------------------------------------------------------------------
mean = 255.0 * np.array([0.485, 0.456, 0.406])
stdev = 255.0 * np.array([0.229, 0.224, 0.225])
normalize = torchvision.transforms.Normalize(mean, stdev)

def preprocess(camera_value):
    global device, normalize
    x = camera_value
    x = cv2.resize(x, (244,244))
    x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB)
    x = x.transpose((2, 0, 1))
    x = torch.from_numpy(x).float()
    x = normalize(x)
    x = x.to(device)
    x = x[None, ...]
    return x

def update(change):
    x = change['new'] 
    x = preprocess(x)
    y = model(x)
    # we apply the `softmax` function to normalize the output vector so it sums to 1 (which makes it a probability distribution)
    y = F.softmax(y, dim=1)
    prob_blocked = float(y.flatten()[0])
    movepub = rospy.Publisher('jetbot_motors/cmd_str', String, queue_size=10)
    if prob_blocked < 0.5:
        movepub.publish(String("backward"))
    else:
        movepub.publish(String("left"))
    time.sleep(0.001)

def callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data ,desired_encoding='passthrough')
    update({'new': cv_image})

# initialization
if __name__ == '__main__':
    # setup ros node
    rospy.init_node('jetbot_collision')

    model = torchvision.models.alexnet(pretrained=False)
    model.classifier[6] = torch.nn.Linear(model.classifier[6].in_features, 2)
    print(os.getcwd())
    model.load_state_dict(torch.load('best_model.pth'))
    device = torch.device('cuda')
    model = model.to(device)

    image_sub = rospy.Subscriber("jetbot_camera/raw",Image,callback)
    # start running
    rospy.spin()
    
