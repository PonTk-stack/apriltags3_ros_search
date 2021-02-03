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
    df = pd.read_csv('mini20210112.csv')
    N = len(df['count'])
    plot_count = df['count']
    t = df['time']
    speed = df['speed_x']
    pos = df['pos_x']
    dt = df['response']
    pixel_w = df['pixel_w']
    pixel_h = df['pixel_h']

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

    pixel = [w*h for w,h in zip( pixel_w,pixel_h)]
    new_speed = [d/ram_t for ram_t,d in zip(dt ,speed)]
    #for w,h in pixel_w,pixel_h:
    #    pixel.append(w*h)




    """"""""""""""""""
    plt.figure(1)
    """"""""""""""""""
    plt.subplot(4,1,1)
    plt.plot(plot_t,plot_speed)
    plt.title("speed_x")
    plt.xlabel("time [s]")
    plt.ylabel("speed [m/s]")
    """"""""""""""""""
    plt.subplot(4,1,2)
    plt.plot(t,pixel)
    plt.ylim(20000,30000)
    plt.title("pixel")
    plt.xlabel("time [s]")
    plt.ylabel("pixel [n]")
    """"""""""""""""""
    plt.subplot(4,1,3)
    plt.plot(plot_count,dt)
    plt.ylim(0,0.01)
    plt.title("sampling time")
    plt.xlabel("count[n]")
    plt.ylabel("time [t]")
    """"""""""""""""""
    plt.subplot(4,1,4)
    plt.plot(t,new_speed)
    plt.title("sampling speed")
    plt.ylim(0,50000)
    plt.xlabel("time[t]")
    plt.ylabel("speed [m/s]")
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
