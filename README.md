# apriltags3_ros_search
I made a ros-node to track AprilTags from camera image.

Before you use this node, you must install some programs.
  * ROS(noetic)
  * OpneCv and cv-bride
  * apriltags_ros

Also if you use algorism of learing in this, you must install some programs(python).
  * gym
  * pytorch
  * numpy
  * pandas
  * scipy
  * matplotlib
  
##How to try 

  `roslaunch tag_trim nodelet_apriltas3.launch`
  `roslaunch tag_trim nodelet_cam.launch`
   
   Also, version of learing in python
   
   `roslaunch tag_tirm learning.launch'
