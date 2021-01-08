#!/usr/bin/env python
# -*- coding: utf-8 -*-
import pandas as pd
import numpy as np
from scipy.fftpack import fft
import matplotlib.pyplot as plt

from kalman_filter import InstantKalmanFilter
from kalman_filter import KalmanFilter


def FourierTransform(f,x,dt,N):
    yf = fft(f)/(len(t)/2) # 離散フーリエ変換&規格化
    freq = np.linspace(0, 1.0/dt, N) # frequency step
    return freq,yf
def LowPassFilter(yf,freq):
    fc = 20  # カットオフ周波数
    yf[(freq > fc)] = 0
    return yf
def InverseFourierTransform(yf,t):
    # 高速逆フーリエ変換（時間信号に戻す）
    f = np.fft.ifft(yf)
    return t,f

def main():
    kf = InstantKalmanFilter()
    kf2 = InstantKalmanFilter()
    kf3 = KalmanFilter()
    kf4 = KalmanFilter()
    df = pd.read_csv('mini20210105.csv')
    N = len(df['count'])
    t = df['time']
    speed = df['speed_x']
    pos = df['pos_x']
    dt = df['response']

    ave_dt = (t[N-1] - t[0])/N

    plot_pos = []
    plot_speed = []
    plot_t = []
    estimated_pos = []
    estimated_speed= []
    speed1 = []
    speed2 = []
    pre_p = 666666
    pre_pp =666666
    
    sum_v = 0
    count = 0
    bunsan = 0
    for z,zz,tt,ddt in zip(pos,speed,t,dt):
        if not (zz == 0.0):

            plot_pos.append(z)
            plot_speed.append(zz)
            plot_t.append(tt)
            pp = kf3.test(z)
            p = z

            i = np.asscalar(pp[0])
            print(i)
            estimated_pos.append(i)
            i = np.asscalar(pp[1])
            estimated_speed.append(i)
            """
            if(pre_pp == 666666):
                pre_p = p
                pre_pp = pp
            """
        else:
            pass



    """"""""""""""""""
    plt.figure(1)
    """"""""""""""""""
    plt.subplot(3,2,1)
    plt.plot(plot_t,plot_pos)
    plt.title("pos_x")
    plt.xlabel("time")
    plt.ylabel("m")
    """"""""""""""""""
    plt.subplot(3,2,2)
    plt.plot(plot_t,estimated_pos)
    plt.title("estimated pos_x")
    plt.xlabel("time")
    plt.ylabel("m")

    """"""""""""""""""
    plt.subplot(3,2,3)
    plt.plot(plot_t,plot_speed)
    plt.title("speed_x")
    plt.xlabel("time")
    plt.ylabel("m")
    """"""""""""""""""
    plt.subplot(3,2,4)
    plt.plot(plot_t,estimated_speed)
    plt.title("estimated speed_x")
    plt.xlabel("time")
    plt.ylabel("m")
    """"""""""""""""""
    """"""""""""""""""
    plt.tight_layout()
    plt.savefig("01")
    plt.show()
if __name__ == '__main__':
    N = 10
    dt = 1
    t = np.arange(0, N*dt, dt) # time
    freq = np.linspace(0, 1.0/dt, N) # frequency step
    main()
