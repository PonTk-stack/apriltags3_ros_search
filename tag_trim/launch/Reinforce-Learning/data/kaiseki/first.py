#!/usr/bin/env python
# -*- coding: utf-8 -*-
import pandas as pd
import numpy as np
from scipy.fftpack import fft
import matplotlib.pyplot as plt



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
    df = pd.read_csv('../mini.csv')
    N = len(df['count'])
    t = df['time']
    f = df['speed_x']

    ave_dt = (t[N-1] - t[0])/N
    print(ave_dt)

    freq,yf = FourierTransform(f,t,ave_dt,N)

    yf2 = LowPassFilter(yf,freq)
    t,f2 = InverseFourierTransform(yf2,t)



    """"""""""""""""""
    plt.figure(4)

    """"""""""""""""""
    plt.subplot(2,2,1)
    plt.plot(t,f)
    plt.title("speed_x")
    plt.ylim(0, 0.02)
    plt.xlabel("time")
    plt.ylabel("m")

    """"""""""""""""""
    plt.subplot(2,2,2)
    plt.plot(freq, np.abs(yf))
    #plt.xlim(0, 100)
#plt.ylim(0, 5)
    plt.title("Fourier")
    plt.xlabel("frequency (Hz)")
    plt.ylabel("signal amplitude")

    """"""""""""""""""
    plt.subplot(2,2,3)
    plt.plot(freq,yf2)
    plt.title("LowPassFilter Fourier")
    plt.xlabel("frequency (Hz)")
    plt.ylabel("signal amplitude")

    """"""""""""""""""
    plt.subplot(2,2,4)
    plt.plot(t,f2)
    plt.title("finally speed_x")
    plt.ylim(0, 0.02)
    plt.xlabel("time")
    plt.ylabel("m")

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
