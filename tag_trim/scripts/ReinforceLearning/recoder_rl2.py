#!/usr/bin/env python3
import time
import numpy as np
import sys

class Recoder_RL2:
    def __init__(self):
        self.filename =  '/home/taisuke/catkin_ws/src/apriltags3_ros_search/tag_trim/launch/Reinforce-Learning/data/aditional/dqn_data/rl2data5.csv'

        col = ['count','detect_count','time','response'\
                ,'tag_velK','anzenK','uv_velK'\
                ,'bbox_lefttop_x','bbox_lefttop_y'\
                ,'bbox_rightbottom_x','bbox_rightbottom_y'\
                ,'pure_bbox_lefttop_x','pure_bbox_lefttop_y'\
                ,'pure_bbox_rightbottom_x','pure_bbox_rightbottom_y'\
                ,'episode','reward']
        self.__f = open(self.filename,'w')
        self.__f.write(self.__list2csv_str(col))
        print("file open :{}".format(self.filename))

        self.reset()
    def __del__(self):
        self.__f.close()

    def reset(self):
        self.count = 0.
        self.detect_count= 0.
        self.time= 0.
        self.response= 0.
        self.tag_velK= 0.
        self.anzenK= 0.
        self.uv_velK= 0.

        self.bbox_lefttop_x = 0
        self.bbox_lefttop_y = 0
        self.bbox_rightbottom_x = 0
        self.bbox_rightbottom_y = 0

        self.pure_bbox_lefttop_x = 0
        self.pure_bbox_lefttop_y = 0
        self.pure_bbox_rightbottom_x = 0
        self.pure_bbox_rightbottom_y = 0

        self.episode= 0.
        self.reward= 0.
    def save(self):
        string = [
            self.count,\
            self.detect_count,\
            self.time,\
            self.response,\
            self.tag_velK,\
            self.anzenK,\
            self.uv_velK,\

            self.bbox_lefttop_x,\
            self.bbox_lefttop_y,\
            self.bbox_rightbottom_x,\
            self.bbox_rightbottom_y,\

            self.pure_bbox_lefttop_x,\
            self.pure_bbox_lefttop_y,\
            self.pure_bbox_rightbottom_x,\
            self.pure_bbox_rightbottom_y,\

            self.episode,\
            self.reward
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

