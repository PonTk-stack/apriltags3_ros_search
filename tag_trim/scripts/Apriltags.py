#!user/bin/env python

import numpy as np
#import quaternion

import sys
from pyquaternion import Quaternion
import time

class Apriltag(object):
    def __init__(self):

        self.id=-1
        self.size = 0.
        self.pose = np.zeros((3,1))
        self.speed = np.zeros((3,1))
        self.accel = np.zeros((3,1))

        self.quaternion = Quaternion(0.,0.,0.,1.) #(w,x,y,z)
        self.d_quaternion = Quaternion(0.,0.,0.,0.) #(w,x,y,z)

        self.euler = np.zeros((1,3))
        self.d_euler = np.zeros((1,3))

        self.pre_speed = np.zeros((3,1))

        self.tt = time.time()
    def set(self, iid, ppos, qq, pre_pos, pre_q, size):
        self.id = iid
        self.size =size
        self.update_pose(ppos,qq)

        self.tt = time.time()
    def update(self, iid, ppos, qq, pre_pos, pre_q):
        if(self.id == iid):
            self.update_velocity(ppos,qq, pre_pos, pre_q)
            self.update_pose(ppos,qq)
            self.tt = time.time()
        else:
            raise valueerror("id is not not miss-match")
            sys.exit()
    def reset_velocity(self):
        self.speed = np.zeros((3,1))
        self.accel = np.zeros((3,1))
        self.d_quaternion = Quaternion(0.,0.,0.,1.) #(w,x,y,z)

    """
    def reset(self, iid, ppos, qq):
        if(self.id == iid):
            self.update_pose(ppos, qq)
            self.reset_velocity()
        else:
            raise ValueError("id is not not miss-match")
            sys.exit()
    """

    def update_pose(self, ppos, qq):
        self.pose = ppos
        self.quaternion = qq

        self.euler = self.Quaternion2Euler(qq)

    def update_velocity(self, ppos, qq, pre_pos, pre_q):
        t = time.time()
        self.pre_speed = self.speed
        self.speed = ppos - pre_pos
        #self.speed = (ppos - pre_pos)/(t-self.tt)
        #self.speed = 0.5*((ppos - pre_pos) + self.pre_speed)

        self.accel = self.speed - self.pre_speed
        #print(self.quaternion.inverse)
        self.d_quaternion = qq * pre_q.inverse

        self.d_euler =(self.Quaternion2Euler(qq)-self.Quaternion2Euler(pre_q))/(t-self.tt)
    def Quaternion2Euler(self,q):
        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        roll = np.arctan(sinr_cosp/ cosr_cosp);
#pitch (y-axis rotation)
        sinp = 2.0 * (q.w * q.y - q.z * q.x);
        if (abs(sinp) >= 1):
            pitch = np.copysign(np.pi / 2, sinp); #use 90 degrees if o    ut of range
        else:
            pitch = np.arcsin(sinp);
#yaw (z-axis rotation)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        yaw = np.arctan(siny_cosp/ cosy_cosp);
        if(cosy_cosp<0):
            if(siny_cosp>=0):
                yaw+=180*np.pi/180
            if(siny_cosp<0):
                yaw-=180*np.pi/180

        #angle=[roll*180/np.pi,pitch*180/np.pi,yaw*180/np.pi]
        return np.array([roll,pitch,yaw])*180/np.pi
