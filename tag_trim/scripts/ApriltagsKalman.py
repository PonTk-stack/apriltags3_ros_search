#!user/bin/env python

import numpy as np
from pyquaternion import Quaternion
from kalman_filter import KalmanFilter
from Apriltags import Apriltag

import time
class ApriltagsKalman(Apriltag,object):
    def __init__(self):
        super(ApriltagsKalman, self).__init__()

        self.kf_x = KalmanFilter()
        self.kf_y = KalmanFilter()
        self.kf_z = KalmanFilter()

    #def set(self, iid, ppos, qq, pre_pos, pre_q, size):
    def update(self, iid, ppos, qq, pre_pos, pre_q):
        if(self.id == iid):
            v = self.update_pose(ppos,qq)
            self.update_velocity(v, qq, pre_q)
        else:
            raise valueerror("id is not not miss-match")
            sys.exit()

    def reset_velocity(self):
        self.speed = np.zeros((3,1))
        self.accel = np.zeros((3,1))
        self.d_quaternion = Quaternion(0.,0.,0.,1.) #(w,x,y,z)

        self.kf_x.reset()
        self.kf_y.reset()
        self.kf_z.reset()

    def update_pose(self, ppos, qq):
        x = self.kf_x.test(ppos[0])
        y = self.kf_y.test(ppos[1])
        z = self.kf_z.test(ppos[2])
        self.pose  = np.array([
            np.asscalar(x[0]),
            np.asscalar(y[0]),
            np.asscalar(z[0])
            ])
        self.quaternion = qq
        return np.array([\
            np.asscalar(x[1]),\
            np.asscalar(y[1]),\
            np.asscalar(z[1])\
            ])

    def update_velocity(self,v, qq, pre_q):
        self.speed = v
        self.d_quaternion = qq * pre_q.inverse

class ApriltagsSpeedKalman(Apriltag,object):
    def __init__(self):
        super(ApriltagsSpeedKalman, self).__init__()

        self.kf_speedx = KalmanFilter()
        self.kf_speedy = KalmanFilter()
        self.kf_speedz = KalmanFilter()

    #def set(self, iid, ppos, qq, pre_pos, pre_q, size):
    def update(self, iid, ppos, qq, pre_pos, pre_q):
        if(self.id == iid):
            self.update_pose(ppos,qq)
            self.update_velocity(ppos,pre_pos, qq, pre_q)
        else:
            raise valueerror("id is not not miss-match")
            sys.exit()

    def reset_velocity(self):
        self.speed = np.zeros((3,1))
        self.accel = np.zeros((3,1))
        self.d_quaternion = Quaternion(0.,0.,0.,1.) #(w,x,y,z)

        self.kf_speedx.reset()
        self.kf_speedy.reset()
        self.kf_speedz.reset()

    def update_pose(self, ppos, qq):
        self.pose  = ppos
        self.quaternion = qq

    def update_velocity(self,ppos,pre_pos, qq, pre_q):
        x = self.kf_speedx.test(ppos[0] - pre_pos[0])
        y = self.kf_speedy.test(ppos[1] - pre_pos[1])
        z = self.kf_speedz.test(ppos[2] - pre_pos[2])
        self.speed  = np.array([\
            np.asscalar(x[0]),\
            np.asscalar(y[0]),\
            np.asscalar(z[0])\
            ])

        self.d_quaternion = qq * pre_q.inverse


if __name__ == '__main__':
    #tag = ApriltagsKalman()
    tag = ApriltagsSpeedKalman()

