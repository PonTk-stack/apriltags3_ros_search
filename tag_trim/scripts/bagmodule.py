#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import sys
import rospy
from std_srvs.srv import SetBool, Trigger, TriggerResponse
from controllable_rosbag_player.srv import Seek, SeekResponse, SetPlaybackSpeed, SetPlaybackSpeedResponse

class BagModule: #client
    def __init__(self):

        rospy.wait_for_service('rosbag_player_controller/play')
        self.player = rospy.ServiceProxy('rosbag_player_controller/play', Trigger)

        rospy.wait_for_service('rosbag_player_controller/pause')
        self.pauser = rospy.ServiceProxy('rosbag_player_controller/pause', Trigger)

    def play(self):
        try:
            return self.player()
        except rospy.ServiceException as  e:
            print ("Service call failed: %s"%e)
    def pause(self):
        try:
            return self.pauser()
        except rospy.ServiceException as  e:
            print ("Service call failed: %s"%e)

if __name__ == "__main__":
    play = True

    bm = BagModule()
    if(play):
        print ("Requesting play")
        print (bm.play())
