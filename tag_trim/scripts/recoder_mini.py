import pandas as pd
import time
import numpy as np

#epis = [x+1 for x in range(len(self.reward_log))]
#data = pd.DataFrame(columns = ['episode', 'reward'])
#data.append(pd.DataFrame({'episode':x, 'reward':self.reward_log}))
#data.to_csv('reward_log.csv')


class Recoder2:
    def __init__(self):
        self.filename = '/home/taisuke/catkin_ws/src/apriltags3_ros_search/tag_trim/launch/Reinforce-Learning/data/mini.csv'
        self.df = pd.DataFrame(columns = \
                ['count','detect_count','time','response'\
                ,'pixel_w','pixel_h'\
                ,'pure_pixel_w','pure_pixel_h'\
                ,'pos_x'\
                ,'speed_x'])

        self.reset()
    def reset(self):
        self.count = 0
        self.detect_count= 0
        self.time= 0.
        self.response= 0.
        self.pixel_w= 0
        self.pixel_h= 0
        self.pure_pixel_w= 0
        self.pure_pixel_h= 0
        self.pos_x = 0
        self.speed_x = 0

        self.count_list       = []
        self.detect_count_list= []
        self.time_list        = []
        self.response_list    = []
        self.pixel_w_list       = []
        self.pixel_h_list       = []
        self.pure_pixel_w_list  = []
        self.pure_pixel_h_list  = []
        self.pos_x_list = []
        self.speed_x_list = []

        self.count_append       = self.count_list.append
        self.detect_count_append= self.detect_count_list.append
        self.time_append        = self.time_list.append
        self.response_append    = self.response_list.append
        self.pixel_w_append       = self.pixel_w_list.append
        self.pixel_h_append       = self.pixel_h_list.append
        self.pure_pixel_w_append  = self.pure_pixel_w_list.append
        self.pure_pixel_h_append  = self.pure_pixel_h_list.append
        self.pos_x_append        = self.pos_x_list.append
        self.speed_x_append        = self.speed_x_list.append

    def save(self):
        i = int(self.count)
        self.df.loc[int(self.count)] =np.array( [\
                self.count,\
                self.detect_count,\
                self.time,\
                self.response,\
                self.pixel_w,\
                self.pixel_h,\
                self.pure_pixel_w,\
                self.pure_pixel_h,\
                self.pos_x,\
                self.speed_x ])

    def to_csv(self):
        self.df.to_csv(self.filename)

    def recode(self):#,count, detect_count, time, response, tag_velK, anzenK, uv_velK, pixel, pure_pixel, episode, reward):
        self.count_append       ( self.count)
        self.detect_count_append( self.detect_count)
        self.time_append        ( self.time)
        self.response_append    ( self.response)
        self.pixel_w_append       ( self.pixel_w)
        self.pixel_h_append       ( self.pixel_h)
        self.pure_pixel_w_append  ( self.pure_pixel_w)
        self.pure_pixel_h_append  ( self.pure_pixel_h)
        self.pos_x_append         ( self.pos_x)
        self.speed_x_append       ( self.speed_x)

if __name__ == '__main__':
    recoder = Recoder()
    recoder.recode()
    recoder.save()
    recoder.to_csv()


