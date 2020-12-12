#!user/bin/env python

import numpy as np
import quaternion

import sys
import tf
from pyquaternion import Quaternion
from uvApriltags import *

class ApriltagsDetector():
    def __init__(self):
        self.tag_list = []
        self.uv_tag_list = []
        self.tag_obj = Apriltag()
        self.uv_tag_obj = UvApriltag()


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
        self.tag_list.clear()


    def getApriltag(self, iid):
        index = self.findID(iid)
        return self.tag_list[index]
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

class Apriltag():
    def __init__(self):

        self.id=-1;
        self.size = 0.;
        self.pose = np.zeros((3,1))
        self.speed = np.zeros((3,1))
        self.accel = np.zeros((3,1))

        self.quaternion = Quaternion(0.,0.,0.,1.) #(w,x,y,z)
        self.d_quaternion = Quaternion(0.,0.,0.,0.) #(w,x,y,z)

        self.pre_speed = np.zeros((3,1))
    def set(self, iid, ppos, qq, pre_pos, pre_q, size):
        self.id = iid
        self.size =size
        self.update_pose(ppos,qq)

    def update(self, iid, ppos, qq, pre_pos, pre_q):
        if(self.id == iid):
            self.update_velocity(ppos,qq, pre_pos, pre_q)
            self.update_pose(ppos,qq)
        else:
            raise ValueError("id is not not miss-match")
            sys.exit()

    def reset(self, iid, ppos, qq):
        if(self.id == iid):
            self.update_pose(ppos, qq)
            self.reset_velocity()
        else:
            raise ValueError("id is not not miss-match")
            sys.exit()

    def update_pose(self, ppos, qq):
        self.pose = ppos
        self.quaternion = qq

    def reset_velocity(self):
        self.speed = np.zeros((3,1))
        self.accel = np.zeros((3,1))
        self.d_quaternion = Quaternion(0.,0.,0.,1.) #(w,x,y,z)

    def update_velocity(self, ppos, qq, pre_pos, pre_q):
        self.pre_speed = self.speed
        self.speed = ppos - pre_pos

        self.accel = self.speed - self.pre_speed
        #print(self.quaternion.inverse)
        self.d_quaternion = qq * pre_q.inverse

