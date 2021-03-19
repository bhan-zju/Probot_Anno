#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time 
import rospy
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray


def hand_camera_callback(msg):
    rospy.loginfo(msg)

def person_subscriber():
	# ROS节点初始化
    rospy.init_node('local_camera_subscriber', anonymous=True)

    rospy.Subscriber("/local_camera/region", Float32MultiArray, hand_camera_callback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    person_subscriber()
