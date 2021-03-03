#!/usr/bin/env python3
import pandas as pd
import time
import numpy as np
import time
import sys

#epis = [x+1 for x in range(len(self.reward_log))]
#data = pd.DataFrame(columns = ['episode', 'reward'])
#data.append(pd.DataFrame({'episode':x, 'reward':self.reward_log}))
#data.to_csv('reward_log.csv')


class Recoder:
    def __init__(self):
        #self.filename =        '/home/taisuke/catkin_ws/src/apriltags3_ros_search/tag_trim/launch/Reinforce-Learning/data/aditional/data11.csv'
        self.filename =        '/home/taisuke/catkin_ws/src/apriltags3_ros_search/tag_trim/launch/Reinforce-Learning/data/aditional/dqn_data/data2.csv'
        col = ['count','detect_count','time','response'\
                ,'tag_velK','anzenK','uv_velK'\
                ,'bbox_lefttop_x','bbox_lefttop_y'\
                ,'bbox_rightbottom_x','bbox_rightbottom_y'\
                ,'pure_bbox_lefttop_x','pure_bbox_lefttop_y'\
                ,'pure_bbox_rightbottom_x','pure_bbox_rightbottom_y'\
                ,'episode','reward']
        self.df = pd.DataFrame(columns = col )

        self.__f = open(self.filename,'w')
        self.__f.write(self.__list2csv_str(col))
        print("file open :{}".format(self.filename))
        
        self.reset()

    def __del__(self):

        self.__f.close()
        print("##########################################################################################################")

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



        self.count_list       = []
        self.detect_count_list= []
        self.time_list        = []
        self.response_list    = []
        self.tag_velK_list    = []
        self.anzenK_list      = []
        self.uv_velK_list     = []

        self.bbox_lefttop_x_list = []
        self.bbox_lefttop_y_list = []
        self.bbox_rightbottom_x_list = []
        self.bbox_rightbottom_y_list = []
        self.pure_bbox_lefttop_x_list = []
        self.pure_bbox_lefttop_y_list = []
        self.pure_bbox_rightbottom_x_list = []
        self.pure_bbox_rightbottom_y_list = []

        self.episode_list     = []
        self.reward_list      = []



        self.count_append       = self.count_list.append
        self.detect_count_append= self.detect_count_list.append
        self.time_append        = self.time_list.append
        self.response_append    = self.response_list.append
        self.tag_velK_append    = self.tag_velK_list.append
        self.anzenK_append      = self.anzenK_list.append
        self.uv_velK_append     = self.uv_velK_list.append

        self.bbox_lefttop_x_append = self.bbox_lefttop_x_list.append
        self.bbox_lefttop_y_append = self.bbox_lefttop_y_list.append
        self.bbox_rightbottom_x_append = self.bbox_rightbottom_x_list.append
        self.bbox_rightbottom_y_append = self.bbox_rightbottom_y_list.append
        self.pure_bbox_lefttop_x_append = self.pure_bbox_lefttop_x_list.append
        self.pure_bbox_lefttop_y_append = self.pure_bbox_lefttop_y_list.append
        self.pure_bbox_rightbottom_x_append = self.pure_bbox_rightbottom_x_list.append
        self.pure_bbox_rightbottom_y_append = self.pure_bbox_rightbottom_y_list.append

        self.episode_append     = self.episode_list.append
        self.reward_append      = self.reward_list.append

    def save(self):
        i = int(self.count)
        self.df.loc[int(self.count)] =np.array( [\
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
                self.reward\
                ])
        self.__save_csv()
    def to_csv(self):
        self.df.to_csv(self.filename)
    def __save_csv(self):
        ss = self.__list2csv_str(self.df.iloc[-1].values)
        self.__f.write(ss)
    def __list2csv_str(self,li):
        ss = ""
        length = len(li)
        for i in range(length):
            ss += str(li[i])
            ss += ',' if (i < length-1) else '\n'
        return ss

    def recode(self):#,count, detect_count, time, response, tag_velK, anzenK, uv_velK, pixel, pure_pixel, episode, reward):
        self.count_append       ( self.count)
        self.detect_count_append( self.detect_count)
        self.time_append        ( self.time)
        self.response_append    ( self.response)
        self.tag_velK_append    ( self.tag_velK)
        self.anzenK_append      ( self.anzenK)
        self.uv_velK_append     ( self.uv_velK)

        self.bbox_lefttop_x_append          (self.bbox_lefttop_x)
        self.bbox_lefttop_y_append          (self.bbox_lefttop_y)
        self.bbox_rightbottom_x_append      (self.bbox_rightbottom_x)
        self.bbox_rightbottom_y_append      (self.bbox_rightbottom_y)
        self.pure_bbox_lefttop_x_append     (self.pure_bbox_lefttop_x)
        self.pure_bbox_lefttop_y_append     (self.pure_bbox_lefttop_y)
        self.pure_bbox_rightbottom_x_append (self.pure_bbox_rightbottom_x)
        self.pure_bbox_rightbottom_y_append (self.pure_bbox_rightbottom_y)

        self.episode_append     ( self.episode)
        self.reward_append      ( self.reward)

if __name__ == '__main__':
    recoder = Recoder()
    recoder.save()
    """
    Recoder()
    """

