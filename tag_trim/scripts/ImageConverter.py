import numpy as np
import cv2

class ImageConverter():
    def imageConvert(self, img,frame):
        conved_img = self.image_trim(img,frame)
        return conved_img
    def image_trim(self,img, frame):
        left = int(frame[0][0])
        upper = int(frame[0][1])
        right = int(frame[1][0])
        lower = int(frame[1][1])
        img_crop = img[upper:lower,left:right]
        #cv2.imshow("img_crop",img_crop)
        #cv2.waitKey(1)
        if len(img.shape)==2:
            rows,cols = img.shape
            conved_img = np.zeros(( rows,cols),np.uint8)
        elif len(img.shape)==3:
            rows,cols,ch = img.shape
            conved_img = np.zeros(( rows,cols,ch ),np.uint8)
        
        conved_img[upper:lower,left:right] = img_crop
        return conved_img
