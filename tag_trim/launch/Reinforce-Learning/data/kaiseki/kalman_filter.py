# -*- coding: utf-8 -*-
import numpy as np
import numpy.random as random
import matplotlib.pyplot as plt

import sys


class KalmanFilter():
    """http://www1.accsnet.ne.jp/~aml00731/kalman.pdf """
    def __init__(self):
        dt = 1.0
        self.I = np.matrix([[1 , 0],
                       [0, 1]]) # 単位行列


        """ 状態方程式"""
        self.x = np.matrix([
            [0],
            [0]
            ]) # 位置と速度: [位置, 加速度]

        self.F = np.matrix([
            [1, dt],
            [0, 1]
            ]) # 運動方程式 [位置+(加速度*時間), 加速度]

        self.G = np.matrix([
            [(dt**2) / 2],
            [dt]
            ]) # 時間遷移に関する雑音モデルの行列 (0平均かつQの正規分布に従う)

        """ 観測方程式"""
        self.P = np.matrix([
            [0, 0],
            [0, 0]
            ]) # 誤差行列
        self.n_P = self.P
        self.H = np.matrix([
            1,
            0
            ]) # 位置のみを線形写像する観測モデル
        #Q = (sigma_a**2) * G * G.T # cov(Gw_k) = (sigma_a)^2 * (G)(G^T): 共分散
        self.Q = 0.01
        #R = sigma_z**2 # R = E(v_k*(v_k)^t) = (sigma_z)^2: ?
        self.R = 0.5

        self.count = 0
    def test(self,z):
        if(self.count ==0):
            x = np.matrix([
                [z],
                [0]
                ]) # 位置と速度: [位置, 加速度]
            self.prepare(x)
        x_n = self.measurement_update(z)
        self.time_update(x_n)
        self.count += 1
        return x_n
    def reset(self):
        self.count = 0
        self.P = np.matrix([
            [0, 0],
            [0, 0]
            ]) # 誤差行列
        self.n_P = self.P

    def prepare(self,new_x):
        if( new_x.shape == self.x.shape ):
            self.x = new_x
        else:
            print(sys.stderr,new_x)
            exit(1)

    def observe(self,z_k):
        return z_k
    def time_update(self,x_n):
        """
        一歩先の状態を予測する
        """
        self.x = (self.F * x_n) #+ (self.G * w) # Fx_{k-1} + Gw_k: 現時刻における予測推定値
        """
        誤差共分散の計算を一歩進める
        """
        self.P = self.F * self.P_n * self.F.T + self.Q # F * P_{k-1} * F^T + G_k * Q_k * (G_k)^T: 現時刻における予測誤差行列

    def measurement_update(self,z):
        """
        kalmanゲインの計算
        """
        S = (self.H * self.P) * self.H.T + self.R # R + H * P_k * H^T: 観測残差の共分散
        K = self.P * self.H.T * S.I # P_k * H^T * S^-1: 最適カルマンゲイン
        """
        観測値Zをもとに推定値の更新
        """
        e = z - self.H * self.x # z_k - H * x_k: 観測残差
        x_n = self.x + K * e # x_k + K_k * e_k: 位置の補正
        #estimate_position.append(x_k_k.tolist()[0][0])
        """
        誤差の共分散の更新
        """
        self.P_n = (self.I - K * self.H) * self.P # (I - K_k * H) * p_k_k: 更新された誤差の共分散
        """"""
        return x_n # estimated

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
        ##################
        gain = P_n / (P_n + self.R )

        x_n = self.x  + gain * (z - self.x )

        """ 更新 """
        self.P = (1 - gain) * P_n
        ##################
        self.x = x_n
        return x_n
if __name__ == '__main__':
    kf = KalmanFilter()
    z = 1.0
    x_n = kf.measurement_update(z)
    kf.time_update(x_n)
    

