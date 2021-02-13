import sys
sys.path.append('/home/taisuke/catkin_ws/src/apriltags3_ros_search/tag_trim/scripts/')
from masking import ImageConverter_ros
from camera import Camera
from LApriltags_ros import LApriltags_ros
import termcolor

class LImageConverter_ros(ImageConverter_ros,object):
    def __init__(self):
        super(LImageConverter_ros,self).__init__()
    def imageConvCallback(self, img,info):
        Camera.setICP(info)
        try:
            image_ori = self.bridge.imgmsg_to_cv2(img,self.encode )
        except CvBridgeError as e:
            print (e)
        #detect
        if(LApriltags_ros.detected_flag):
            #frame = [[0,0],[1280,720]]
            frame = LApriltags_ros.frame
            conved_img = self.imageConvert(image_ori,frame)
            img_msg = self.image2msg(conved_img)
            #cv2.imshow("conved_img",conved_img)
            #cv2.waitKey(1)
        #nondetect
        else:
            conved_img = image_ori  #frame = [[0,0],[1280,720]]
            img_msg = self.image2msg(conved_img)
        self.publishProcess(img_msg, info )
