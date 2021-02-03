import pandas as pd
import time
import numpy as np

#epis = [x+1 for x in range(len(self.reward_log))]
#data = pd.DataFrame(columns = ['episode', 'reward'])
#data.append(pd.DataFrame({'episode':x, 'reward':self.reward_log}))
#data.to_csv('reward_log.csv')


class Recoder2:
    def __init__(self):
        self.filename = '/home/taisuke/catkin_ws/src/apriltags3_ros_search/tag_trim/launch/Reinforce-Learning/data/powerpo6_mini.csv'
        col =  ['count','detect_count','time','response'\
                ,'pixel_w','pixel_h'\
                ,'pure_pixel_w','pure_pixel_h'\
                ,'pos_x' \
                ,'pos_y' \
                ,'pos_z' \
                ,'speed_x'\
                ,'speed_y'\
                ,'speed_z'\
                ,'euler_x' \
                ,'euler_y' \
                ,'euler_z' \
                ,'speed_eulerx'\
                ,'speed_eulery'\
                ,'speed_eulerz'\
                ]

        self.__f = open(self.filename,'w')
        self.__f.write(self.__list2csv_str(col))
        print("file open :{}".format(self.filename))
        self.df = pd.DataFrame(columns = col)

        self.reset()
    def __del__(self):
        self.__f.close()

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
        self.pos_y =0
        self.pos_z = 0
        self.speed_x =0
        self.speed_y=0
        self.speed_z = 0
        self.euler_x =0
        self.euler_y =0
        self.euler_z=0
        self.speed_eulerx=0
        self.speed_eulery=0
        self.speed_eulerz=0

        self.count_list       = []
        self.detect_count_list= []
        self.time_list        = []
        self.response_list    = []
        self.pixel_w_list       = []
        self.pixel_h_list       = []
        self.pure_pixel_w_list  = []
        self.pure_pixel_h_list  = []

        self.pos_x_list = []
        self.pos_y_list = []
        self.pos_z_list = []
        self.speed_x_list = []
        self.speed_y_list = []
        self.speed_z_list = []
        self.euler_x_list = []
        self.euler_y_list = []
        self.euler_z_list = []
        self.speed_eulerx_list = []
        self.speed_eulery_list = []
        self.speed_eulerz_list = []

        self.count_append       = self.count_list.append
        self.detect_count_append= self.detect_count_list.append
        self.time_append        = self.time_list.append
        self.response_append    = self.response_list.append
        self.pixel_w_append       = self.pixel_w_list.append
        self.pixel_h_append       = self.pixel_h_list.append
        self.pure_pixel_w_append  = self.pure_pixel_w_list.append
        self.pure_pixel_h_append  = self.pure_pixel_h_list.append
        self.pos_x_append        = self.pos_x_list.append
        self.pos_y_append        = self.pos_y_list.append
        self.pos_z_append        = self.pos_z_list.append
        self.speed_x_append        = self.speed_x_list.append
        self.speed_y_append        = self.speed_y_list.append
        self.speed_z_append        = self.speed_z_list.append
        self.euler_x_append        = self.euler_x_list.append
        self.euler_y_append        = self.euler_y_list.append
        self.euler_z_append        = self.euler_z_list.append
        self.speed_eulerx_append = self.speed_eulerx_list.append
        self.speed_eulery_append = self.speed_eulery_list.append
        self.speed_eulerz_append = self.speed_eulerz_list.append

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
                self.pos_y ,\
                self.pos_z ,\
                self.speed_x,\
                self.speed_y,\
                self.speed_z,\
                self.euler_x,\
                self.euler_y,\
                self.euler_z,\
                self.speed_eulerx,\
                self.speed_eulery,\
                self.speed_eulerz
                
                ])

        #self.__save_csv()

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
        self.pixel_w_append       ( self.pixel_w)
        self.pixel_h_append       ( self.pixel_h)
        self.pure_pixel_w_append  ( self.pure_pixel_w)
        self.pure_pixel_h_append  ( self.pure_pixel_h)

        self.pos_x_append        (self.pos_x)
        self.pos_y_append        (self.pos_y)
        self.pos_z_append        (self.pos_z)
        self.speed_x_append      (self.speed_x)
        self.speed_y_append      (self.speed_y)
        self.speed_z_append      (self.speed_z)
        self.euler_x_append      (self.euler_x)
        self.euler_y_append      (self.euler_y)
        self.euler_z_append      (self.euler_z)
        self.speed_eulerx_append (self.speed_eulerx)
        self.speed_eulery_append (self.speed_eulery)
        self.speed_eulerz_append (self.speed_eulerz)

if __name__ == '__main__':
    recoder = Recoder2()
    recoder.save()


