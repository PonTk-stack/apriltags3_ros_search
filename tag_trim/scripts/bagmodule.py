#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rosbag
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from scipy.sparse import csr_matrix

"""
for topic, msg, t in bag.read_messages():
    i+=1
    if(i == 5): exit(0)
    if topic == "/usb_cam/camera_info":
        A = msg.P
        A =np.array([[msg.P[0],msg.P[1],msg.P[2]],
                [msg.P[3],msg.P[4],msg.P[5]],
                [msg.P[6],msg.P[7],msg.P[8]]], dtype=np.uint8)
        dist = np.array([msg.D[:]], dtype=np.uint8)
    if topic == "/usb_cam/image_raw":
        image_raw = bridge.imgmsg_to_cv2(msg, "bgr8")
        image_ori = cv2.undistort(image_raw, A, dist, None) # 内部パラメータを元に画像補正
"""
        




class Rosbag():
    def __init__(self):
        self.bag = rosbag.Bag(\
        '/home/taisuke/catkin_ws/cam_bag/1m/2020-09-28-13-32-02.bag')
    def __del__(self):
        self.bag.close()


    def load_bag(self):
        for topic, msg, t in bag.read_messages():
            if topic == "/usb_cam/camera_info":
                #A = msg.P
                #dist = msg.K
                A =np.array([[msg.P[0],msg.P[1],msg.P[2]],
                        [msg.P[3],msg.P[4],msg.P[5]],
                        [msg.P[6],msg.P[7],msg.P[8]]], dtype=np.uint8)
                dist = np.array([msg.D[:]], dtype=np.uint8)
            if topic == "/usb_cam/image_raw":
                image_raw = bridge.imgmsg_to_cv2(msg, "bgr8")
                image_ori = cv2.undistort(image_raw, A, dist, None) # 内部パラメータを元に画像補正



