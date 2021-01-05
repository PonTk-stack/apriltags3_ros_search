# -*- coding: utf-8 -*-
import numpy as np
import numpy.random as random
import matplotlib.pyplot as plt



class KalmanFilter():
    def __init__(self):

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
        n_P = P
        self.H = np.matrix([
            1,
            0
            ]) # 位置のみを線形写像する観測モデル
        #Q = (sigma_a**2) * G * G.T # cov(Gw_k) = (sigma_a)^2 * (G)(G^T): 共分散
        self.Q = 0.01
        #R = sigma_z**2 # R = E(v_k*(v_k)^t) = (sigma_z)^2: ?
        self.R = 0.5

    def observe(self,z_k):
        return z_k

    def time_update(self,x_n):
        """
        一歩先の状態を予測する
        """
        self.x = (self.F * x_n) + (G * w) # Fx_{k-1} + Gw_k: 現時刻における予測推定値
        """
        誤差共分散の計算を一歩進める
        """
        P_n = F * P * F.T + Q # F * P_{k-1} * F^T + G_k * Q_k * (G_k)^T: 現時刻における予測誤差行列

    def measurement_update(self):
        """
        kalmanゲインの計算
        """
        S = (H * P) * H.T + R # R + H * P_k * H^T: 観測残差の共分散
        K = P * H.T * S.I # P_k * H^T * S^-1: 最適カルマンゲイン
        """
        観測値Zをもとに推定値の更新
        """
        e = z - H * x # z_k - H * x_k: 観測残差
        x_n = x + K * e # x_k + K_k * e_k: 位置の補正
        #estimate_position.append(x_k_k.tolist()[0][0])
        """
        誤差の共分散の更新
        """
        P_n = (I - K * H) * P # (I - K_k * H) * p_k_k: 更新された誤差の共分散
        return x_n # estimated
    def test(self,z):
        self.time_update()
        estimated = self.measurement_update(z)

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









