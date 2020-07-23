#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

bridge = CvBridge()

def callback(data):
    try:
        cv_image = bridge.imgmsg_to_cv2(data,"bgr8")
    except CvBridgeError as e:
        print(e)
    img_title = 'photo.jpg'
    cv2.imwrite(img_title, cv_image)

def callbackD(data):
    try:
        cv_depth = bridge.imgmsg_to_cv2(data)
    except CvBridgeError as e:
        print(e)
    img_title = 'photo(d).jpg'
    #cv_depth = cv_depth.astype('uint8')
    #print(cv_depth[0])
    #print(cv_depth.dtype)
    cv2.imwrite(img_title, cv_depth)

def take_photo():
    rospy.init_node('take_photo', anonymous=True)

    img_topic = "/camera/rgb/image_raw"
    image_sub = rospy.Subscriber(img_topic, Image, callback)
    depth_topic = "/camera/depth/image_raw"
    depth_image_sub = rospy.Subscriber(depth_topic, Image, callbackD)
    r = rospy.Rate(10)
    r.sleep()
    rospy.spin()

if __name__ == '__main__':
    take_photo()
