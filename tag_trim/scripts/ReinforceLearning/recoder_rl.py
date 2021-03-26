#!/usr/bin/env python3
import pandas as pd
import time
import numpy as np
import sys

import os

class Recoder_RL:
    def __init__(self):
        self.filename =   os.environ["HOME"]+"/tag_trim_RL_data/data/aditional/dqn_data/rldata10.csv"
        col = ['count','detect_count'\
                ,'episode','reward','reward_ave','aveloss','learn_count']
        self.__f = open(self.filename,'w')
        self.__f.write(self.__list2csv_str(col))
        print("file open :{}".format(self.filename))

        self.reset()
    def __del__(self):
        self.__f.close()

    def reset(self):
        self.count = 0.
        self.detect_count= 0.
        self.episode= 0.
        self.reward= 0.
        self.reward_ave= 0.
        self.loss= 0.
        self.learn_count = 0.
    def save(self):
        string = [
            self.count,
            self.detect_count,
            self.episode,
            self.reward,
            self.reward_ave,
            self.loss,
            self.learn_count
        ]
        self.__save_csv(string)
    def __save_csv(self,string):
        ss = self.__list2csv_str(string)
        self.__f.write(ss)
        self.__f.flush()

    def __list2csv_str(self,li):
        ss = ""
        length = len(li)
        for i in range(length):
            ss += str(li[i])
            ss += ',' if (i < length-1) else '\n'
        return ss
if __name__ == '__main__':
    recoder = Recoder_RL()
    recoder.save()
