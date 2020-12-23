#!user/bin/env python
import numpy as np

import sys
from pyquaternion import Quaternion
from uvApriltags import *
from Apriltags import *

class ApriltagsDetector():
    def __init__(self):
        self.tag_list = []
        self.uv_tag_list = []
        self.tag_obj = Apriltag()
        self.uv_tag_obj = UvApriltag()

    def setGain(self,tag_velK=1.0,anzenK=1.5,uv_velK=0.1):
        UvApriltag.tag_velK = tag_velK
        UvApriltag.anzenK = anzenK
        UvApriltag.uv_velK = uv_velK
    def getGain(self):
        return UvApriltag.tag_velK,UvApriltag.anzenK,UvApriltag.uv_velK
    def setApriltag(self, detect):
        iid = detect.id[0]
        pose = np.array([[   detect.pose.pose.pose.position.x,\
                            detect.pose.pose.pose.position.y,\
                            detect.pose.pose.pose.position.z ]]).T

        pre_pose=np.array([[detect.pre_pose.pose.position.x,\
                            detect.pre_pose.pose.position.y,\
                            detect.pre_pose.pose.position.z ]]).T

        q =Quaternion(np.array([    detect.pose.pose.pose.orientation.w,\
                                    detect.pose.pose.pose.orientation.x,\
                                    detect.pose.pose.pose.orientation.y,\
                                    detect.pose.pose.pose.orientation.z ])\
                                    )
        pre_q =Quaternion(np.array([detect.pre_pose.pose.orientation.w,\
                                    detect.pre_pose.pose.orientation.x,\
                                    detect.pre_pose.pose.orientation.y,\
                                    detect.pre_pose.pose.orientation.z ])\
                                    )

        size = detect.size[0]
        index = self.findID(iid)
        if(index >= 0):
            self.updateApriltag(self.tag_list[index],iid,pose,q,pre_pose,pre_q)
        else:
            tag = Apriltag()
            tag.set(iid,pose,q,pre_pose,pre_q, size)
            self.tag_list.append(tag)

    def reset_tag_vels(self,ids):
        for tag in self.tag_list:
            if not(tag.id in ids):
                tag.reset_velocity()
    def all_clear_tags(self):
        #self.tag_list.clear()
        del self.tag_list[:]


    def getApriltag(self, iid):
        index = self.findID(iid)
        return self.tag_list[index]
    def _getUvPureApriltagSize(self,iid):
        index = self.findID(iid)
        #size = self.uv_tag_obj.tagPose2pure_uv_size(self.tag_list[index])
        return self.uv_tag_obj.tagPose2pure_uv_size(self.tag_list[index])
    def getUvPureApriltagSize(self,detect):
        iid = detect.id[0]
        index = self.findID(iid)
        size = self.uv_tag_obj.tagPose2pure_uv_size(self.tag_list[index])
        return size
    def getUvApriltag(self,detect):
        iid = detect.id[0]
        index = self.findID(iid)
        frame = self.uv_tag_obj.tagPose2uv(self.tag_list[index])
        return frame

    def updateApriltag(self, tag, iid, pose, q, pre_pos, pre_q):
        tag.update(iid,pose,q,pre_pos,pre_q)

    def findID(self, iid):
        finded = False
        for index, tag_obj in enumerate(self.tag_list):
            if (iid == tag_obj.id):
                finded = True
                break
        if(finded):
            return index
        else:
            #raise ValueError("findID: id is not not miss-match")
            return -1

