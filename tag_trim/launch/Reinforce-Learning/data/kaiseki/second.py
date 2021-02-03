#!/usr/bin/env python
# -*- coding: utf-8 -*-
import pandas as pd
import numpy as np
from scipy.fftpack import fft
import matplotlib.pyplot as plt

from kalman_filter import InstantKalmanFilter


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
    df = pd.read_csv('mini20210105.csv')
    N = len(df['count'])
    t = df['time']
    speed = df['speed_x']
    pos = df['pos_x']
    dt = df['response']

    ave_dt = (t[N-1] - t[0])/N

    estimated_pos = []
    estimated_t = []
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
            pp = kf.test(z)
            p = z
            estimated_pos.append(pp)
            estimated_speed.append(kf2.test(zz))
            estimated_t.append(tt)
            if(pre_pp == 666666):
                pre_p = p
                pre_pp = pp
            v = (p - pre_p)
            sum_v += v
            count +=1
            bunsan += (v-0.7407281779429095 )**2
            speed1.append(v)

            v = (pp-pre_pp)/ddt
            speed2.append(v)
            pre_pp = pp
            pre_p = z
        else:
            pass

    print("bunsan_v", bunsan/count)
    print("hyoujun_v", np.sqrt(bunsan/count))
    print("ave_v", sum_v/count)


    """"""""""""""""""
    plt.figure(1)
    """"""""""""""""""
    plt.subplot(3,2,1)
    plt.plot(t,pos)
    plt.title("pos_x")
    plt.xlabel("time")
    plt.ylabel("m")
    """"""""""""""""""
    plt.subplot(3,2,2)
    plt.plot(estimated_t,estimated_pos)
    plt.title("estimated pos_x")
    plt.xlabel("time")
    plt.ylabel("m")

    """"""""""""""""""
    plt.subplot(3,2,3)
    plt.plot(t,speed)
    plt.ylim(0.,0.03)
    plt.title("speed_x")
    plt.xlabel("time")
    plt.ylabel("m")
    """"""""""""""""""
    plt.subplot(3,2,4)
    plt.plot(estimated_t,estimated_speed)
    plt.ylim(0.,0.03)
    plt.title("estimated speed_x")
    plt.xlabel("time")
    plt.ylabel("m")
    """"""""""""""""""
    plt.subplot(3,2,5)
    plt.plot(estimated_t,speed1)
    plt.title(" speed1_x")
    plt.xlabel("time")
    plt.ylabel("m/s")
    """"""""""""""""""
    plt.subplot(3,2,6)
    plt.plot(estimated_t,speed2)
    plt.title(" speed2_x")
    plt.xlabel("time")
    plt.ylabel("m/s")


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
