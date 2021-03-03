#!/bin/bash

s=0.0
u=22.8
duration=22.796139

#s=1.5
#u=19.0
#duration=24.948483

rate=2.0
#hz="scale=3; 1/$duration"| bc
hz=$(echo "scale=9; $rate / $duration" | bc)
rosrun rosbag_manager rosbag_manager_node  -s $s -u $u   /home/taisuke/catkin_ws/cam_bag/naname_up/2021-01-14-13-27-29.bag -r $rate  --clock  --hz=$hz $1
#rosbag play -s $s -u $u 2021-01-14-13-44-22.bag -r $rate  --clock  --hz=$hz $1
