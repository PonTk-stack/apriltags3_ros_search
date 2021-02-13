#!/usr/bin/env python3
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

import sys
sys.path.append('/home/taisuke/catkin_ws/src/roscpp_Manager/rosbag_manager/scripts/')
from bag_clock_counter_ros import BagClockCounter_ros


class Ltag_trim():
    def __init__(self):
        self.nodename ="Ltag_trim_node"
        rospy.init_node(self.nodename)

        self.bcc = BagClockCounter_ros()
        self.lapriltags_ros = LApriltags_ros(self.bcc)
        self.licr = LImageConverter_ros()

    def main(self):
        rospy.spin()

if __name__ == "__main__":
    this_node = Ltag_trim()
    try:
        #sim = Learning()
        #sim.load()
        #ipr = ImagePubliser()
        #ipr.load()
        rospy.spin()
        #this_node.main()
    except rospy.ROSInterruptException:
        pass



