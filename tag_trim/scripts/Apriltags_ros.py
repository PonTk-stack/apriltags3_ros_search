#!user/bin/env python
from camera import *
import numpy as np
from  ApriltagsDetector import *
import rospy
from apriltag_ros.msg import AprilTagDetectionArray

class Apriltags_ros():
    detected_flag = False
    frame = np.array([[0,0],[Camera.image_size[0],Camera.image_size[1]]])
    frames = []

    def __init__(self, callback = True):
        if(callback):
            self.sub_tag =rospy.Subscriber('/tag_topic',AprilTagDetectionArray,self.tagDetectedCallback)
        self.tag_detector = ApriltagsDetector()

        self.count = 0
    def __del__(self):
        print("owari")
    def tagDetectedCallback(self,msg):
        ids = []
        #Apriltags_ros.frames.clear()
        Apriltags_ros.frames =[]
        if len(msg.detections)>0:
            self.count +=1
            print(self.count)
            Apriltags_ros.detected_flag = True

            for i in range(len(msg.detections)):
                ids.append(msg.detections[i].id[0])
                self.tag_detector.setApriltag(msg.detections[i])

            self.tag_detector.reset_tag_vels(ids)
            for i in range(len(msg.detections)):
                Apriltags_ros.frame=self.tag_detector.getUvApriltag(msg.detections[i])
                Apriltags_ros.frames.append(Apriltags_ros.frame)
        else:
            self.tag_detector.all_clear_tags()
            Apriltags_ros.detected_flag = False
            self.tag_detector.reset_tag_vels(ids)
            Apriltags_ros.frame = np.array([ 
                [0,0],
                [Camera.image_size[0],Camera.image_size[1]]
                ])
            Apriltags_ros.frames.append(Apriltags_ros.frame)
