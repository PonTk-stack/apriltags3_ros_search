#!/usr/bin/env python
import rosbag
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from scipy.sparse import csr_matrix

from LApriltags_ros import *
from LImageConverter_ros import *

import rospy
from sensor_msgs.msg import Image, CameraInfo
import cv2
import time



class Ltag_trim:
    def __init__(self):
        self.nodename ="Ltag_trim_node"
        rospy.init_node(self.nodename)

        lapriltags_ros = LApriltags_ros()
        licr = LImageConverter_ros()

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    this_node = Ltag_trim()
    try:
        #sim = Learning()
        #sim.load()
        #ipr = ImagePubliser()
        #ipr.load()
        this_node.main()
    except rospy.ROSInterruptException:
        pass





class ImagePubliser():
    def __init__(self):
        self.image_pub = rospy.Publisher("usb_cam/image", Image, queue_size = 1)
        self.info_pub = rospy.Publisher("usb_cam/camera_info", CameraInfo, queue_size = 1)

        self.bag = rosbag.Bag(\
        '/home/taisuke/catkin_ws/cam_bag/1m/2020-09-28-13-32-02.bag')
        self.bridge = CvBridge()
        self.t  = 0
    def load(self):
        for topic, msg, t in self.bag.read_messages():

            if topic == "/usb_cam/camera_info":
                #A = msg.P
                #dist = msg.K
                A =np.array([[msg.P[0],msg.P[1],msg.P[2]],
                        [msg.P[3],msg.P[4],msg.P[5]],
                        [msg.P[6],msg.P[7],msg.P[8]]], dtype=np.uint8)
                dist = np.array([msg.D[:]], dtype=np.uint8)
            if topic == "/usb_cam/image_raw":
                image_raw = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                #image_ori = cv2.undistort(image_raw, A, dist, None)
                cv2.imshow("aa",image_raw)
                dt =(t- self.t)
                dt = int(dt.to_sec()*10000)
                if(dt == 0): dt=1
                cv2.waitKey(dt)
            self.t = t
