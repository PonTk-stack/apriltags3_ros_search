#!/usr/bin/env python
import rospy
import numpy as np
import math
import tf
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray
import geometry_msgs.msg
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3
import tf_conversions
from geometry_msgs.msg import Quaternion
import tf2_ros
import rosparam

class Tag_listener(object):

  def __init__(self):
    self._sub_tag = rospy.Subscriber('/tag_detections',AprilTagDetectionArray,self._callback_tag)
    self._tag_position = np.zeros(4, dtype = 'float64')
    self._listener = tf.TransformListener()
    self.br = tf.TransformBroadcaster()
    self.br2 = tf2_ros.TransformBroadcaster()

    #rosparam.set_param("/apriltags3_listener", "value")

  def _callback_tag(self,messege):
    abc = rospy.get_param("apriltags3_param")
    if len(messege.detections) > 0:
      angle_q = messege.detections[0].pose.pose.pose.orientation
      angle_r = self._change_angle([angle_q.x,angle_q.y,angle_q.z,angle_q.w])
      self._tag_position[0] = messege.detections[0].pose.pose.pose.position.x
      self._tag_position[1] = messege.detections[0].pose.pose.pose.position.y*(1.0)
      self._tag_position[2] = messege.detections[0].pose.pose.pose.position.z
      self._tag_position[3] = math.degrees(angle_r[2])
#print(messege.detections[0].id[0])
      self.handle_pose(messege.detections[0].id[0])
#else:
# print "nothing"
  def _change_angle(self,quaternion):
    #External parameters
    e = tf.transformations.euler_from_quaternion(quaternion)    #change angle
    return e

  def handle_pose(self,id_):
#        t = tf.Transformer(True, rospy.Duration(10.0))
#        m = geometry_msgs.msg.TransformStamped()
#        m.header.frame_id = "THISFRAME"
#        m.parent_id = "PARENT"
#        t.setTransform(m)
    if(id_==0):#robo_Golf
          robo_high=0.188

#        if(id_==1):#EDM_Golf
#            robo_high=0


    t=TransformStamped();
    t.header.stamp=rospy.Time.now()
    t.header.frame_id="ZMP"
    t.child_frame_id="camera"
    cam_high=0.865
    translation =Vector3(0,0,cam_high-robo_high)
    t.transform.translation=translation
    q=tf_conversions.transformations.quaternion_from_euler(-1.*(90+0.)*3.14/180,0,0,"sxyz")
    rotation = Quaternion(*q)
    t.transform.rotation = rotation
    self.br2.sendTransform(t)

    try:
      (trans1,rot1) = self._listener.lookupTransform('ZMP', 'tag_0', rospy.Time(0))
      print ('trans1:',trans1)
      print ('rot1:',rot1)
    except:
      pass


if __name__ == "__main__":
  rospy.init_node('apriltags3_listener',anonymous=True)
  tag_listener = Tag_listener()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    pass

