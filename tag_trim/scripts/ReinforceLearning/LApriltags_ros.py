#!/usr/bin/env python                                                 
import rospy
from rosgraph_msgs.msg import Clock
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from apriltag_ros.msg import AprilTagDetectionArray

import termcolor
import cv2
import numpy as np
from pyquaternion import Quaternion
import time
#######################################################
import os
import sys
sys.path.append('/home/taisuke/catkin_ws/src/apriltags3_ros_search/tag_trim/scripts')
from ImageConverter import *
from camera import *
from ApriltagsDetector import *
from Apriltags_ros import *
from time_checker import TCR
from recoder import Recoder
recoder = Recoder()
from recoder_mini import Recoder2
recoder2 = Recoder2()
#######################################################
import sys
#sys.path.append('/home/taisuke/catkin_ws/src/roscpp_Manager/rosbag_manager/scripts/')
#from bag_clock_counter import BagClockCounter

from env import Environment
from q_learning import QLearningAgent





class LApriltags_ros(Apriltags_ros, object):
    contact_img_converter = True
    detected_flag = False
    frame = np.array([[0,0],[Camera.image_size[0],Camera.image_size[1]]])
    pure_frame_sizes = []

    def __init__(self,bcc):
        super(LApriltags_ros, self).__init__(callback=False)
        self.sub_tag =rospy.Subscriber('/tag_topic',AprilTagDetectionArray,self.LtagDetectedCallback)
        #self.tag_detector = ApriltagsDetector()
        self.bcc = bcc

        self.Learning_init()
        self.detected   =  0
        self.nodetected =  0
        self.go_learn = False

        self.setup_time = self.begin =self.now = self.pre_time= time.time()
        self.TCR = TCR()

        self.pre_detected_flag = False
        self.pre_pure_pixel =0
        self.pre_pixel=0
        self.lean_count=0
    def __del__(self):
        recoder.to_csv()
    def LtagDetectedCallback(self,msg):
        self.TCR.begin()
        if(self.bcc.need_switch_fase()):
            print(termcolor.colored("switch",'red'))
            self.TCR.reset()

            self.lean_count += 1
            if(self.lean_count %3==0):
                recoder.to_csv()
                self.Learning_reset()
            self.go_learn = False
            #LApriltags_ros.detected_flag = False
            recoder.to_csv()
            recoder.reset()
            recoder2.to_csv()
            recoder2.reset()
            self.begin = time.time()

        ids = []
        #LApriltags_ros.pure_frame_sizes.clear()
        LApriltags_ros.pure_frame_sizes = []
        if len(msg.detections)>0:
            if(self.TCR.between() >=1.0 ):
                print(termcolor.colored(self.TCR.between(),'magenta'))
#                self.go_learn = True
            LApriltags_ros.detected_flag = True

            for i in range(len(msg.detections)):
                ids.append(msg.detections[i].id[0])
                self.tag_detector.setApriltag(msg.detections[i])

            self.tag_detector.reset_tag_vels(ids)
            for i in range(len(msg.detections)):
                iid = msg.detections[i].id[0]
                pure_frame_size=self.tag_detector._getUvPureApriltagSize(iid)
                LApriltags_ros.frame=self.tag_detector.getUvApriltag(msg.detections[i])
                #LApriltags_ros.pure_frame_sizes.append(self.pure_frame_size)
                #LApriltags_ros.frames.append(Apriltags_ros.frame)
            print(termcolor.colored("detect",'blue'))
        else:

            self.tag_detector.all_clear_tags()
            LApriltags_ros.detected_flag = False
            self.tag_detector.reset_tag_vels(ids)
            pure_frame_size = np.array([0,0])
            LApriltags_ros.frame = np.array([
                [0,0],
                [Camera.image_size[0],Camera.image_size[1]]
                ])
            #LApriltags_ros.pure_frame_sizes.append(self.pure_frame_size)
#            LApriltags_ros.frames.append(Apriltags_ros.frame)
            print(termcolor.colored("nondetect",'yellow'))
        self.detect_count(LApriltags_ros.detected_flag,self.TCR.response())
        if(self.go_learn):
            self.setGainWithLearning(pure_frame_size,LApriltags_ros.frame)
##########################measure############################
        recoder.time = self.TCR.now()
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
        self.pre_pixel=w*h
        w,h =  pure_frame_size
        recoder.pure_pixel = w*h
        recoder.pure_pixel_w = w
        recoder.pure_pixel_h = h
        self.pre_pure_pixel =w*h
        recoder.episode = self.QLagent.episode
        recoder.reward = self.QLagent.reward

        recoder.recode()
        recoder.save()
##########################measure2############################
        recoder2.time = self.TCR.now()
        w = rightbottom[0] - lefttop[0]
        h = rightbottom[1] - lefttop[1]
        recoder2.pixel_w = w
        recoder2.pixel_h = h
        w,h =  pure_frame_size
        recoder2.pure_pixel_w = w
        recoder2.pure_pixel_h = h
        self.pre_pure_pixel =w*h
        try:
            recoder2.pos_x = self.tag_detector.getApriltag(0).pose[0][0]
        except:
            recoder2.pos_x = 0

        try:
            recoder2.speed_x = self.tag_detector.getApriltag(0).speed[0][0]
        except:
            recoder2.speed_x = 0

        recoder2.recode()
        recoder2.save()
############################################################

        self.TCR.end()
        self.pre_detected_flag  = LApriltags_ros.detected_flag
    def detect_count(self , detect_flag, response):
        recoder.count += 1
        recoder.response = response
        recoder2.count += 1
        recoder2.response = response
        if(detect_flag):
            recoder.detect_count += 1
            recoder2.detect_count += 1
            self.detected  +=  1
            self.nodetected =  0
        else:
            self.detected   =  0
            self.nodetected += 1

    def setGainWithLearning(self,pure_frame_size,frame):
            pure_pixel = pure_frame_size[0]*pure_frame_size[1] #w,h =  pure_frame_size
            #lefttop,rightbottom = frame
            pixel =\
                    (frame[1][0] - frame[0][0])\
                    *(frame[1][1] - frame[0][1])#(rightbottom[0]-lefttop[0])*(rightbottom[1]-lefttop[1])
            k2, k3 = self.Learning(LApriltags_ros.detected_flag\
                    ,pure_pixel,pixel)
#            print("anzenK : {}, uv_velK : {} , reward : {}".format(k2, k3, self.QLagent.reward))
            if not(k2 == k3 ==0):
                self.tag_detector.setGain(anzenK=k2,uv_velK=k3 )

    def Learning_init(self):
        tag_velK,anzenK,uv_velK=self.tag_detector.getGain()
        env = Environment(anzenK,uv_velK)
        self.QLagent = QLearningAgent(env)
    def Learning_reset(self):
        self.QLagent.reset_episode()
    def Learning(self,detect_flag, pure_pixel, pixel, count = 2):
        if(self.detected >= count or self.nodetected==1):
            return self.QLagent.learn(detect_flag\
                    , pure_pixel, pixel) #anzenk, uv_velk
        else:
            return [0,0]

    def get_pure_frame_sizes(self ):
        return LApriltags_ros.frame_sizes


