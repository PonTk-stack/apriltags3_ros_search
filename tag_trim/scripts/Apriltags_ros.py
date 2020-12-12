#!user/bin/env python

class Apriltags_ros():
    def __init__(self):
        self.sub_tag =rospy.Subscriber('/tag_pose',apriltagsArray,self.callback_tag)
    def callback_tag(self,msg):
        if len(msg.detections)>0:

