#!/usr/bin/env python3                                               
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
sys.path.append(os.environ["HOME"]+'/catkin_ws/src/apriltags3_ros_search/tag_trim/scripts')
from ImageConverter import *
from camera import *
from ApriltagsDetector import *
from Apriltags_ros import *
from time_checker import TCR

#######################################################
import sys
#from bag_clock_counter import BagClockCounter

#from env import Environment
#from q_learning import QLearningAgent

from env2 import Environment2
from dqn_agent import DQN
import matplotlib.pyplot as plt
import matplotlib
import torch
from recoder_rl import Recoder_RL
recoder_rl = Recoder_RL()
from recoder_rl2 import Recoder_RL2
recoder_rl2 = Recoder_RL2()


class LApriltags_ros(Apriltags_ros,object):
    contact_img_converter = True
    detected_flag = False
    frame = np.array([[0,0],[Camera.image_size[0],Camera.image_size[1]]])
    pure_frame_sizes = []

    def __init__(self,bcc,set_learn = True):
        super(LApriltags_ros, self).__init__(callback=False)
        self.bcc = bcc
        #self.tag_detector = ApriltagsDetector()

        self._Learning_init()

        self.__detected   =  0
        self.__nodetected =  0

        self.set_learn = set_learn
        self.__go_learn = False

        self.sub_tag =rospy.Subscriber('/tag_topic',AprilTagDetectionArray,self.LtagDetectedCallback)
        self.setup_time = self.begin =self.now = self.pre_time= time.time()
        self.TCR = TCR()


        self.count = 0
        self.err_count = 0
        self.detect_count = 0
        self.__continuous_detect_count = 0

        self.pure_frame_size = np.array([0,0])
        self.__pure_frame = np.array([[0,0],[0,0]])
    def LtagDetectedCallback(self,msg):
        ##########################pre_measure############################
        response = self.TCR.response()
        lefttop,rightbottom = LApriltags_ros.frame
        pure_lefttop, pure_rightbottom = self.__pure_frame
        ##########################measure2############################
        """
        recoder2.count += 1
        recoder2.time = self.TCR.now()
        recoder2.response = response
        w = rightbottom[0] - lefttop[0]
        h = rightbottom[1] - lefttop[1]
        recoder2.pixel_w = w
        recoder2.pixel_h = h
        w,h =  self.pure_frame_size
        recoder2.pure_pixel_w = w
        recoder2.pure_pixel_h = h
        try:
            recoder2.pos_x = self.tag_detector.getApriltag(0).pose[0][0]
            recoder2.pos_y = self.tag_detector.getApriltag(0).pose[1][0]
            recoder2.pos_z = self.tag_detector.getApriltag(0).pose[2][0]
            recoder2.speed_x = self.tag_detector.getApriltag(0).speed[0][0]
            recoder2.speed_y = self.tag_detector.getApriltag(0).speed[1][0]
            recoder2.speed_z = self.tag_detector.getApriltag(0).speed[2][0]
            recoder2.euler_x = self.tag_detector.getApriltag(0).euler[0]
            recoder2.euler_y = self.tag_detector.getApriltag(0).euler[1]
            recoder2.euler_z = self.tag_detector.getApriltag(0).euler[2]
            recoder2.speed_eulerx = self.tag_detector.getApriltag(0).d_euler[0]
            recoder2.speed_eulery = self.tag_detector.getApriltag(0).d_euler[1]
            recoder2.speed_eulerz = self.tag_detector.getApriltag(0).d_euler[2]
        except:
            recoder2.pos_x = 0
            recoder2.pos_y = 0
            recoder2.pos_z = 0
            recoder2.speed_x = 0
            recoder2.speed_y = 0
            recoder2.speed_z = 0
            recoder2.euler_x = 0
            recoder2.euler_y = 0
            recoder2.euler_z = 0
            recoder2.speed_eulerx = 0
            recoder2.speed_eulery = 0
            recoder2.speed_eulerz = 0

        recoder2.save()
        """
        ##########################measure3############################
        recoder_rl2.time = self.TCR.between()
        recoder_rl2.response = response
        tag_velK,anzenK,uv_velK=self.tag_detector.getGain()
        recoder_rl2.tag_velK = tag_velK
        recoder_rl2.anzenK = anzenK
        recoder_rl2.uv_velK = uv_velK

        recoder_rl2.bbox_lefttop_x = lefttop[0]
        recoder_rl2.bbox_lefttop_y = lefttop[1]
        recoder_rl2.bbox_rightbottom_x = rightbottom[0]
        recoder_rl2.bbox_rightbottom_y = rightbottom[1]
        recoder_rl2.pure_bbox_lefttop_x = pure_lefttop[0]
        recoder_rl2.pure_bbox_lefttop_y = pure_lefttop[1]
        recoder_rl2.pure_bbox_rightbottom_x = pure_rightbottom[0]
        recoder_rl2.pure_bbox_rightbottom_y = pure_rightbottom[1]

        recoder_rl2.episode = self._episode
        recoder_rl2.reward = self._episode_reward

        recoder_rl2.save()


        ###########################!--measure--#########################
        if(self.bcc.need_switch_fase()):
            recoder_rl.episode = self._episode
            recoder_rl.reward = self._episode_reward
            if self._episode_learn_count == 0:
                recoder_rl.loss = None
            else:
                recoder_rl.loss = self._episode_loss/self._episode_learn_count
            recoder_rl.learn_count = self._episode_learn_count
            recoder_rl.save()
            """
            recoder.to_csv()
            recoder2.to_csv()
            recoder.reset()
            recoder2.reset()
            """
            #print(termcolor.colored("switch",'red'))
            self.TCR.reset()

            self._Learning_reset()
            self.__go_learn = False
            self.begin = time.time()
            self.count = 0
            self.err_count = 0
            self.detect_count = 0
            self.__continuous_detect_count = 0

            #LApriltags_ros.detected_flag = False
        """
        > detect_t-1 > learning_t-1 > k_t-1 > frame_t
        >  detect_t  >  learning_t  >  k_t  > frame_t+1
        > detect_t+1 > learning_t+1 > k_t+1 > frame_t+2
        """
        ##########################learning###########################
        if(self.set_learn  and self.__go_learn):
            tag_velK,anzenK,uv_velK=self.tag_detector.getGain()
            k2,k3 = self._GainWithLearning((anzenK,uv_velK),LApriltags_ros.detected_flag,\
                    self.__pure_frame,LApriltags_ros.frame)
            self.tag_detector.setGain(anzenK=k2,uv_velK=k3 )
            if not (LApriltags_ros.detected_flag):
                self.__go_learn = False
        #######################!--learning--#########################

        ids = []
        #LApriltags_ros.pure_frame_sizes.clear()
        LApriltags_ros.pure_frame_sizes = []
        if len(msg.detections)>0:
            self.__continuous_detect_count += 1
            if(self.__continuous_detect_count >=2 and self.TCR.between() >=1.0):
                self.__go_learn = True

            LApriltags_ros.detected_flag = True

            for i in range(len(msg.detections)):
                ids.append(msg.detections[i].id[0])
                self.tag_detector.setApriltag(msg.detections[i])

            self.tag_detector.reset_tag_vels(ids)
            for i in range(len(msg.detections)):
                iid = msg.detections[i].id[0]
                self.pure_frame_size=self.tag_detector._getUvPureApriltagSize(iid)
                LApriltags_ros.frame, self.__pure_frame = self.tag_detector.getUvApriltag(msg.detections[i])
                #LApriltags_ros.pure_frame_sizes.append(self.pure_frame_size)
                #LApriltags_ros.frames.append(Apriltags_ros.frame)
            #print(termcolor.colored("detect"+str(self.count),'blue'))
        else:

            self.__continuous_detect_count = 0
            self.tag_detector.all_clear_tags()
            LApriltags_ros.detected_flag = False
            self.tag_detector.reset_tag_vels(ids)
            self.pure_frame_size = np.array([0,0])
            self.__pure_frame = np.array([[0,0],[0,0]])
            LApriltags_ros.frame = np.array([
                [0,0],
                [Camera.image_size[0],Camera.image_size[1]]
                ])
            #LApriltags_ros.pure_frame_sizes.append(self.pure_frame_size)
#            LApriltags_ros.frames.append(Apriltags_ros.frame)
            self.err_count+=1
            #print(termcolor.colored("nondetect"+str(self.err_count),'yellow'))

        self.count += 1
        self.recode_count(LApriltags_ros.detected_flag)

    def recode_count(self , detect_flag):
        recoder_rl.count = self.count
        recoder_rl2.count = self.count
        if(detect_flag):
            self.detect_count += 1
            recoder_rl.detect_count = self.detect_count
            recoder_rl2.detect_count = self.detect_count
            self.__detected  +=  1
            self.__nodetected =  0
        else:
            self.__detected   =  0
            self.__nodetected += 1

    def _GainWithLearning(self,state,detected_flag,pure_frame,frame):
            k2, k3 = self._Learning(state,detected_flag\
                                ,pure_frame,frame)
            return k2, k3

    def _Learning_init(self):

        #tag_velK,anzenK,uv_velK=self.tag_detector.getGain()

        '''
        env = Environment(anzenK,uv_velK)
        self.QLagent = QLearningAgent(env)
        '''
        self.env = Environment2()
        self.dqn_agent = DQN(self.env.observation_space, self.env.action_space)

        self._episode = -1
        self._episode_reward = None
        self.__episode_reward_store = []
        #self.__episode_reward_store.append(self._episode_reward)
        self._Learning_reset()
    def _Learning_reset(self):
        '''
        self.QLagent.reset_episode()
        '''
        self._episode += 1

        print(termcolor.colored("Episode: "+str(self._episode)+" , reward: "+str(self._episode_reward),'yellow'))
        self._episode_reward = 0.
        self._episode_learn_count = 0
        self._episode_loss = 0.
        s = self.env.reset()
        return s

    def _Learning(self,state,detect_flag, pure_frame, frame):
        s = state
        a = self.dqn_agent.choose_action(s)
        n_s, r, done, info = self.env.step(s,a)

        pure_lt, pure_rb = pure_frame
        lt,rb = frame #lefttop,rightbottom
        if(lt[0] < pure_lt[0]\
            and pure_rb[0] < rb[0]\
            and lt[1] < pure_lt[1]\
            and pure_rb[1] < rb[1]\
            ):
            pure_frame_in_frame = 1.0
        else:
            pure_frame_in_frame = 0.0

        pixel  = (rb[0]-lt[0])*(rb[1]-lt[1])
        pure_pixel= (pure_rb[0]-pure_lt[0])*(pure_rb[1]-pure_lt[1])
        if detect_flag:
            r1 =pure_frame_in_frame*\
                   100.* (1.-((pixel-pure_pixel)/pixel))
        else:
            r1 = -2.

        r =  r1# + (r2 + r3)
        if done:
            self.env.reset()
            r = -0.001

            r2 = -2.
            r3 = -2
        else:
            anzenK,uv_velK = n_s
            minth , maxth = self.env._k1_threshold
            r2 = (maxth-anzenK)/(maxth-minth)
            minth , maxth = self.env._k2_threshold
            r3 = (maxth-uv_velK)/(maxth-minth)

        #if not detect_flag : r=-0.1
        print(r,info)
        self.dqn_agent.store_transition(s, a, r, n_s)
        self._episode_reward += r

        if(self.dqn_agent.memory_counter > self.dqn_agent._MEMORY_CAPACITY):
            self._episode_loss += self.dqn_agent.learn()
            self._episode_learn_count += 1
            if done:
                #print("Ep : ",self._episode," r: ",self._episode_reward)
                ppp=0
        return n_s

        """
        if(self.__detected >= count or self.__nodetected==1):
            return self.QLagent.learn(detect_flag\
                    , pure_pixel, pixel) #anzenk, uv_velk
        else:
            return [0,0]
        """
    def plot_reward(self):
        plt.figure(1)
        plt.clf()
        sum_reward =torch.tensor(self.__episode_reward_store,dtype=torch.float)
        plt.title('Training...')
        plt.xlabel('Episode')
        plt.ylabel('reward')
        plt.plot(sum_reward.numpy())
        if len(sum_reward) > 100.:
            means = sum_reward.unfold(0.,100.,1.).mean(1).view(-1)
            means = torch.cat((torch.zeros(99),means))
            plt.plot(means.numpy())
        plt.pause(0.01)

        if self.is_ipython:
            display.clear_output(wait=True)
            display.display(plt.gcf())

