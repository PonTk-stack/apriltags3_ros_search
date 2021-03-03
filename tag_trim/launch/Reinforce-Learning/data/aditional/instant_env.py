#!/usr/bin/env python
# -*- coding: utf-8 -*-
import pandas as pd

from gym import spaces, logger



from observer import Observer

import sys
sys.path.append('/home/taisuke/catkin_ws/src/apriltags3_ros_search/tag_trim/scripts/ReinforceLearning/')
from state import Action

class InstantTagObsever(Observer):
    def transform(self,state):
        return np.array(state).reshape((1,-1))
    

class InstantEnvironment:
    def __init__(self):

        self.__name = "data"  # .csv

        self.state = None
        self._index = 0  # seconds between state updates
        self.steps_beyond_done = None

    def get_target_file(self, num):
        self.__filenum = num
        target_file = self.__name + str(self.__filenum) + ".csv"

        df = pd.read_csv(target_file)

        self.t = df['time']
        self.count = df['count']
        self.detect_count = df['detect_count']
        self.k_tagvel = df['tag_velK']
        self.k_anzen = df['anzenK']
        self.k_uvvel = df['uv_velK']

        self.bbox_lt_x = df['bbox_lefttop_x']
        self.bbox_lt_y = df['bbox_lefttop_y']
        self.bbox_rb_x = df['bbox_rightbottom_x']
        self.bbox_rb_y = df['bbox_rightbottom_y']

        self.pure_bbox_lt_x = df['pure_bbox_lefttop_x']
        self.pure_bbox_lt_y = df['pure_bbox_lefttop_y']
        self.pure_bbox_rb_x = df['pure_bbox_rightbottom_x']
        self.pure_bbox_rb_y = df['pure_bbox_rightbottom_y']

    def step(self):
        self._index += 1
        k_1 = self.k_anzen[self._index]
        k_2 = self.k_uvvel[self._index]

        self.state = (k_1, k_2)
        done = bool(  self._index >=len(self.t)  )
        if not done:
            reward = self.detect_count
        elif self.steps_beyond_done is None:
            self.steps_beyond_done = 0
            reward = self.detect_count
        else:
            if self.steps_beyond_done == 0:
                logger.warn( "tukareta")
            self.steps_beyond_done += 1
            reward = 0.0
        return s , r , done, {}

    def reset(self):
        self._index =0
        self.steps_beyond_done = None

    @property
    def action_space(self):
        return spaces.Discrete(4)

if __name__ == "__main__":
    env = InstantTagObsever(InstantEnvironment())
    print(env.action_space.n)
