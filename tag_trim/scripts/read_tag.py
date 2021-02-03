#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray
from os import system



class Tag_listener(object):
  def __init__(self):
    self._sub_tag = rospy.Subscriber('/tag_detections',AprilTagDetectionArray,self._callback_tag)
    self._tag_position = np.zeros(4, dtype = 'float64')
    self._listener = tf.TransformListener()
  def _callback_tag(self,messege):
    if len(messege.detections) > 0:
      angle_q = messege.detections[0].pose.pose.pose.orientation
      angle_r = self._change_angle([angle_q.x,angle_q.y,angle_q.z,angle_q.w])
      self._tag_position[0] = messege.detections[0].pose.pose.pose.position.x
      self._tag_position[1] = messege.detections[0].pose.pose.pose.position.y
      self._tag_position[2] = messege.detections[0].pose.pose.pose.position.z
      self._tag_position[3] = math.degrees(angle_r[2])
      system("clear")
      rospy.loginfo("x : %s",self._tag_position[0])
      rospy.loginfo("y : %s",self._tag_position[1])
      rospy.loginfo("z : %s",self._tag_position[2])
      rospy.loginfo("sita_x : %s",angle_r[0]*180./np.pi)
      rospy.loginfo("sita_y : %s",angle_r[1])
      rospy.loginfo("sita_z : %s",angle_r[2])
      try:
        (trans1,rot1) = self._listener.lookupTransform('usb_cam', 'tag0', rospy.Time(0))
        print 'tf:',trans1
        print 'rot1:',rot1
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
    else:
      print "nothing"
  def _change_angle(self,quaternion):
    #External parameters
    e = tf.transformations.euler_from_quaternion(quaternion)  #change angle
    return e
if __name__ == "__main__":
  rospy.init_node('tag_listener')
  tag_listener = Tag_listener()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    pass
