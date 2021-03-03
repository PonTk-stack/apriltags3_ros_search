#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt




def main():
    df = pd.read_csv('rldata5.csv')
    N = df['count']
    detectN = df['detect_count']
    episode = df['episode']
    sumreward = df['reward']
    aveloss = df['aveloss']
    lean_count = df['learn_count']

#    df.plot()
#    plt.show()

    plot_N = []
    plot_detectN = []
    plot_episode = []
    plot_sumreward = []
    plot_aveloss = []
    for n ,den,epi,sumr,aloss in zip(N,detectN,episode,sumreward,aveloss):
        plot_N.append(n)

        plot_detectN.append(den)
        plot_episode.append(epi)
        plot_sumreward.append(sumr)
        if(aloss == 'None'):
            plot_aveloss.append(0.)
        else:
            plot_aveloss.append(float(aloss))

        #i = np.asscalar(pp[1])

    """"""""""""""""""
    plt.figure(1,figsize=(14.0,6.0))
    """"""""""""""""""
    plt.subplot(1,1,1)
    plt.plot(plot_episode,plot_sumreward)
    plt.title("reward_graph")
    #plt.xlim(2,12)
    #plt.xticks(x_ran)
    plt.grid(color='b',linestyle='dotted',linewidth=0.5)
    plt.minorticks_on()
    plt.xlabel("episode")
    plt.ylabel("sum_reward")
    """"""""""""""""""

    """"""""""""""""""
    #plt.tight_layout()
    #plt.savefig("01")
    plt.show()

if __name__ == '__main__':
    main()
