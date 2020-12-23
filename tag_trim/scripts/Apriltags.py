#!user/bin/env python

import numpy as np
#import quaternion

import sys
from pyquaternion import Quaternion

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

