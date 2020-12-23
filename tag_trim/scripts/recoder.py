import pandas as pd
import time
import numpy as np

#epis = [x+1 for x in range(len(self.reward_log))]
#data = pd.DataFrame(columns = ['episode', 'reward'])
#data.append(pd.DataFrame({'episode':x, 'reward':self.reward_log}))
#data.to_csv('reward_log.csv')


class Recoder:
    def __init__(self):
        self.filename = '/home/taisuke/catkin_ws/src/apriltags3_ros_search/tag_trim/launch/Reinforce-Learning/data/trim_data.csv'
        self.df = pd.DataFrame(columns = \
                ['count','detect_count','time','response'\
                ,'tag_velK','anzenK','uv_velK'\
                ,'pixel','pixel_w','pixel_h'\
                ,'pure_pixel','pure_pixel_w','pure_pixel_h'\
                ,'episode','reward'])

        self.count = 0
        self.detect_count= 0
        self.time= 0.
        self.response= 0.
        self.tag_velK= 0.
        self.anzenK= 0.
        self.uv_velK= 0.
        self.pixel= 0
        self.pixel_w= 0
        self.pixel_h= 0
        self.pure_pixel= 0
        self.pure_pixel_w= 0
        self.pure_pixel_h= 0
        self.episode= 0
        self.reward= 0.

        self.count_list       = []
        self.detect_count_list= []
        self.time_list        = []
        self.response_list    = []
        self.tag_velK_list    = []
        self.anzenK_list      = []
        self.uv_velK_list     = []
        self.pixel_list       = []
        self.pixel_w_list       = []
        self.pixel_h_list       = []
        self.pure_pixel_list  = []
        self.pure_pixel_w_list  = []
        self.pure_pixel_h_list  = []
        self.episode_list     = []
        self.reward_list      = []

        self.count_append       = self.count_list.append
        self.detect_count_append= self.detect_count_list.append
        self.time_append        = self.time_list.append
        self.response_append    = self.response_list.append
        self.tag_velK_append    = self.tag_velK_list.append
        self.anzenK_append      = self.anzenK_list.append
        self.uv_velK_append     = self.uv_velK_list.append
        self.pixel_append       = self.pixel_list.append
        self.pixel_w_append       = self.pixel_w_list.append
        self.pixel_h_append       = self.pixel_h_list.append
        self.pure_pixel_append  = self.pure_pixel_list.append
        self.pure_pixel_w_append  = self.pure_pixel_w_list.append
        self.pure_pixel_h_append  = self.pure_pixel_h_list.append
        self.episode_append     = self.episode_list.append
        self.reward_append      = self.reward_list.append
    def reset(self):
        self.count = 0.
        self.detect_count= 0.
        self.time= 0.
        self.response= 0.
        self.tag_velK= 0.
        self.anzenK= 0.
        self.uv_velK= 0.
        self.pixel= 0.
        self.pixel_w= 0.
        self.pixel_h= 0.
        self.pure_pixel= 0.
        self.pure_pixel_w= 0.
        self.pure_pixel_h= 0.
        self.episode= 0.
        self.reward= 0.

        self.count_list       = []
        self.detect_count_list= []
        self.time_list        = []
        self.response_list    = []
        self.tag_velK_list    = []
        self.anzenK_list      = []
        self.uv_velK_list     = []
        self.pixel_list       = []
        self.pixel_w_list       = []
        self.pixel_h_list       = []
        self.pure_pixel_list  = []
        self.pure_pixel_w_list  = []
        self.pure_pixel_h_list  = []
        self.episode_list     = []
        self.reward_list      = []

        self.count_append       = self.count_list.append
        self.detect_count_append= self.detect_count_list.append
        self.time_append        = self.time_list.append
        self.response_append    = self.response_list.append
        self.tag_velK_append    = self.tag_velK_list.append
        self.anzenK_append      = self.anzenK_list.append
        self.uv_velK_append     = self.uv_velK_list.append
        self.pixel_append       = self.pixel_list.append
        self.pixel_w_append       = self.pixel_w_list.append
        self.pixel_h_append       = self.pixel_h_list.append
        self.pure_pixel_append  = self.pure_pixel_list.append
        self.pure_pixel_w_append  = self.pure_pixel_w_list.append
        self.pure_pixel_h_append  = self.pure_pixel_h_list.append
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
                self.pixel,\
                self.pixel_w,\
                self.pixel_h,\
                self.pure_pixel,\
                self.pure_pixel_w,\
                self.pure_pixel_h,\
                self.episode,\
                self.reward ])

    def to_csv(self):
        self.df.to_csv(self.filename)

    def recode(self):#,count, detect_count, time, response, tag_velK, anzenK, uv_velK, pixel, pure_pixel, episode, reward):
        self.count_append       ( self.count)
        self.detect_count_append( self.detect_count)
        self.time_append        ( self.time)
        self.response_append    ( self.response)
        self.tag_velK_append    ( self.tag_velK)
        self.anzenK_append      ( self.anzenK)
        self.uv_velK_append     ( self.uv_velK)
        self.pixel_append       ( self.pixel)
        self.pixel_w_append       ( self.pixel_w)
        self.pixel_h_append       ( self.pixel_h)
        self.pure_pixel_append  ( self.pure_pixel)
        self.pure_pixel_w_append  ( self.pure_pixel_w)
        self.pure_pixel_h_append  ( self.pure_pixel_h)
        self.episode_append     ( self.episode)
        self.reward_append      ( self.reward)

if __name__ == '__main__':
    recoder = Recoder()
    recoder.recode()
    recoder.save()
    recoder.to_csv()


