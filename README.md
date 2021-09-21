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
  
  
##Dependent libraries

　try
 
  `$ cd ~/catkin_ws/src`
  `$ git clone https://github.com/PonTk-stack/File-Manager.git`
  `$ cd ~/catkin_ws`
  `$ catkin_make -j4`

##試してみる方法 

  `$ roslaunch tag_trim nodelet_apriltas3.launch`  
  `$ roslaunch tag_trim nodelet_cam.launch`  
   
  Also, version of learing in python  
  
  `$ roslaunch tag_tirm learning.launch`  
。
