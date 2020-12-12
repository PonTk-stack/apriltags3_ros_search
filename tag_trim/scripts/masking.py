#!/usr/bin/env python                                                 
import rospy
import sys
import cv2
import message_filters
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np
from Apriltags import ApriltagsDetector
from ImageConverter import * 

from camera import *

from pyquaternion import Quaternion

class Apriltags_ros():
    detected_flag = False
    frame = np.array([[0,0],[Camera.image_size[0],Camera.image_size[1]]])
    frames = []

    def __init__(self):
        self.sub_tag =rospy.Subscriber('/tag_topic',AprilTagDetectionArray,self.tagDetectedCallback)
        self.tag_detector = ApriltagsDetector()
    def tagDetectedCallback(self,msg):
        ids = []
        Apriltags_ros.frames.clear()
        if len(msg.detections)>0:
            Apriltags_ros.detected_flag = True

            for i in range(len(msg.detections)):
                ids.append(msg.detections[i].id[0])
                self.tag_detector.setApriltag(msg.detections[i])

            self.tag_detector.reset_tag_vels(ids)
            for i in range(len(msg.detections)):
                self.tag_detector.all_clear_tags()
                Apriltags_ros.frame=self.tag_detector.getUvApriltag(msg.detections[i])
                Apriltags_ros.frames.append(Apriltags_ros.frame)
        else:
            Apriltags_ros.detected_flag = False
            self.tag_detector.reset_tag_vels(ids)
            Apriltags_ros.frame = np.array([ 
                [0,0],
                [Camera.image_size[0],Camera.image_size[1]]
                ])
            Apriltags_ros.frames.append(Apriltags_ros.frame)

class ImageConverter_ros(ImageConverter):
    def __init__(self):
        sub1 = message_filters.Subscriber("image_topic", Image)
        sub2 = message_filters.Subscriber("camera_info_topic", CameraInfo)
        ts = message_filters.ApproximateTimeSynchronizer([sub1,sub2], 1, 0.5)
        ts.registerCallback(self.imageConvCallback)
        self.image_pub = rospy.Publisher("tag_trim_node/image_trimmed", Image, queue_size=1)
        self.info_pub = rospy.Publisher("tag_trim_node/camera_info", CameraInfo, queue_size=1)

        self.bridge=CvBridge()
    def __del__(self):
        cv2.destroyAllWindows()
    def imageConvCallback(self, img,info):
        Camera.setICP(info)
        try:
            image_ori = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError, e:
            print e
        #detect
        if(Apriltags_ros.detected_flag):
            #frame = [[0,0],[1280,720]]
            frame = Apriltags_ros.frame
            conved_img = self.imageConvert(image_ori,frame)
            img_msg = self.image2msg(conved_img)
            #cv2.imshow("conved_img",conved_img)
            #cv2.waitKey(1)
        #nondetect
        else:
            conved_img = image_ori  #frame = [[0,0],[1280,720]]
            img_msg = self.image2msg(conved_img)
        self.publishProcess(img_msg, info )
    def image2msg(self, image):
        return self.bridge.cv2_to_imgmsg(image, "bgr8")

    def publishProcess(self,img_msg,info_msg):
        now = rospy.Time.now()
        img_msg.header.stamp = now
        info_msg.header.stamp = now
        self.image_pub.publish(img_msg)
        self.info_pub.publish(info_msg)



class tag_trim:
    def __init__(self):
        self.nodename = "tag_trim_node"
        rospy.init_node(self.nodename)
#       sub1 = message_filters.Subscriber('/usb_cam/image_raw', Image)

        icr = ImageConverter_ros()
        apriltags_ros = Apriltags_ros()
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    this_node = tag_trim()
    try:
        this_node.main()

    except rospy.ROSInterruptException:
        pass





