#!/usr/bin/env python                                                 
import rospy
from rosgraph_msgs.msg import Clock
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from apriltag_ros.msg import AprilTagDetectionArray

import cv2
import numpy as np
from pyquaternion import Quaternion
import time
#######################################################
import sys
sys.path.append('/home/taisuke/catkin_ws/src/apriltags3_ros_search/tag_trim/scripts')
from ImageConverter import *
from camera import *
from ApriltagsDetector import *
from Apriltags_ros import *
from recoder import Recoder
recoder = Recoder()
#######################################################
import sys
sys.path.append('/home/taisuke/catkin_ws/src/roscpp_Manager/rosbag_manager/scripts/')
from bag_clock_counter import BagClockCounter

from env import Environment
from q_learning import QLearningAgent



class BagClockCounter_ros(BagClockCounter,object):
    def __init__(self):
        super(BagClockCounter_ros, self).__init__()
        self.sub =rospy.Subscriber('/clock',Clock,self.counterCallback)


class LApriltags_ros(Apriltags_ros, object):
    detected_flag = False
    frame = np.array([[0,0],[Camera.image_size[0],Camera.image_size[1]]])
    pure_frame_sizes = []

    def __init__(self):
        super(LApriltags_ros, self).__init__(callback=False)
        
        self.sub_tag =rospy.Subscriber('/tag_topic',AprilTagDetectionArray,self.LtagDetectedCallback)
        #self.tag_detector = ApriltagsDetector()
        self.pure_frame_size = np.array([0,0])

        self.bcc = BagClockCounter_ros()
        self.Learning_init()
        self.detected   =  0
        self.nodetected =  0
        self.go_learn = False

        self.begin =self.now = self.pre_time= time.time()
    def __del__(self):
        recoder.to_csv()
    def LtagDetectedCallback(self,msg):
        ids = []
        #LApriltags_ros.pure_frame_sizes.clear()
        LApriltags_ros.pure_frame_sizes = []
        if len(msg.detections)>0:
            self.go_learn = True
            LApriltags_ros.detected_flag = True

            for i in range(len(msg.detections)):
                ids.append(msg.detections[i].id[0])
                self.tag_detector.setApriltag(msg.detections[i])

            self.tag_detector.reset_tag_vels(ids)
            for i in range(len(msg.detections)):
                iid = msg.detections[i].id[0]
                self.pure_frame_size=self.tag_detector._getUvPureApriltagSize(iid)
                LApriltags_ros.frame=self.tag_detector.getUvApriltag(msg.detections[i])
                #LApriltags_ros.pure_frame_sizes.append(self.pure_frame_size)
                #LApriltags_ros.frames.append(Apriltags_ros.frame)
        else:
            self.tag_detector.all_clear_tags()
            LApriltags_ros.detected_flag = False
            self.tag_detector.reset_tag_vels(ids)
            self.pure_frame_size = np.array([0,0])
            LApriltags_ros.frame = np.array([ 
                [0,0],
                [Camera.image_size[0],Camera.image_size[1]]
                ])
            #LApriltags_ros.pure_frame_sizes.append(self.pure_frame_size)
#            LApriltags_ros.frames.append(Apriltags_ros.frame)
        self.detect_count(LApriltags_ros.detected_flag)
        if(self.go_learn):
            k2, k3 = self.Learning(LApriltags_ros.detected_flag)
            print("reward is {}".format(self.QLagent.reward))
            if not(k2 == k3 ==0):
                self.tag_detector.setGain(anzenK=k2,uv_velK=k3 )
##########################measure############################
        self.now = time.time()
        tag_velK,anzenK,uv_velK=self.tag_detector.getGain()
        recoder.tag_velK = tag_velK
        recoder.anzenK = anzenK
        recoder.uv_velK = uv_velK
        lefttop,rightbottom = LApriltags_ros.frame
        w = rightbottom[0] - lefttop[0]
        h = rightbottom[1] - lefttop[1]
        recoder.pixel = w*h
        recoder.pixel_w = w
        recoder.pixel_h = h
        w,h =  self.pure_frame_size
        recoder.pure_pixel = w*h
        recoder.pure_pixel_w = w
        recoder.pure_pixel_h = h
        recoder.episode = self.QLagent.episode
        recoder.reward = self.QLagent.reward

        recoder.recode()
        recoder.save()
############################################################
        if(self.bcc.need_switch_fase()):
            self.Learning_reset()
            self.go_learn = False
            #recoder.reset()
            recoder.to_csv()
            self.begin = time.time()

        self.pre_time = time.time()

    def detect_count(self , detect_flag):
        recoder.count += 1
        recoder.time = self.now - self.begin
        recoder.response = self.now - self.pre_time
        if(detect_flag):
            recoder.detect_count += 1
            self.detected  +=  1
            self.nodetected =  0
        else:
            self.detected   =  0
            self.nodetected += 1
    def Learning_init(self):
        tag_velK,anzenK,uv_velK=self.tag_detector.getGain()
        env = Environment(anzenK,uv_velK)
        self.QLagent = QLearningAgent(env)
    def Learning_reset(self):
        self.QLagent.reset_episode()
    def Learning(self,detect_flag, count = 2):
        if(self.detected >= count or self.nodetected==1):
            return self.QLagent.learn(detect_flag) #anzenk, uv_velk
        else:
            return 0,0

    def get_pure_frame_sizes(self ):
        return LApriltags_ros.frame_sizes


