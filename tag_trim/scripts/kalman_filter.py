# -*- coding: utf-8 -*-
import numpy as np
import numpy.random as random

class InstantKalmanFilter():
    def __init__(self):
        self.x = -0.0000011
        self.P = 0
        self.Q = 0.0075
        self.R = 0.532
    def test(self, z):
        if(self.x == -0.0000011):
            self.x = z
        P_n  = self.P + self.Q
        gain = P_n / (P_n + self.R )
        x_n = self.x  + gain * (z - self.x )
        """ 更新 """  
        self.P = (1 - gain) * P_n
        self.x = x_n
        return x_n
