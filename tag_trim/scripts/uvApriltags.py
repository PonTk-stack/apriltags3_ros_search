import numpy as np
from camera import *
import termcolor

class UvApriltag(Camera):
    tag_velK = 1.0
    anzenK = 1.5
    uv_velK = 0.1
    def __init__(self):

        self.ones = np.array([[1.0,1.0,1.0,1.0]])
        self.sizeM = np.array([[0.5,-0.5,-0.5, 0.5],\
                               [0.5, 0.5,-0.5,-0.5],\
                               [0.0, 0.0, 0.0, 0.0] ])

        self.pre_uv = np.zeros((3,1))
    def tagPose2pure_uv_size(self, apriltag):
        pose = apriltag.pose
        predict_pose = pose + (UvApriltag.tag_velK*apriltag.speed)
        uv = np.dot(Camera.A,predict_pose)
        uv /= uv[2,0]
        ts = apriltag.size

        q = apriltag.quaternion
        R = q.rotation_matrix

        posM = np.dot(Camera.A,np.dot(predict_pose,self.ones)+np.dot(np.linalg.inv(R),ts*self.sizeM))
        p1 = np.array([[posM[0,0],posM[1,0]]]).T/posM[2,0]
        p2 = np.array([[posM[0,1],posM[1,1]]]).T/posM[2,1]
        p3 = np.array([[posM[0,2],posM[1,2]]]).T/posM[2,2]
        p4 = np.array([[posM[0,3],posM[1,3]]]).T/posM[2,3]

        pure_wh = self.cat_pure_wh(p1,p2,p3,p4)
        return pure_wh 


    def tagPose2uv(self, apriltag):
        pose = apriltag.pose
        speed = apriltag.speed

        predict_pose = pose + (UvApriltag.tag_velK*speed)
#        uv = Camera.A*predict_pose.T
        uv = np.dot(Camera.A,predict_pose)
        uv /= uv[2,0]

        ts = apriltag.size

        q = apriltag.quaternion
        R = q.rotation_matrix

        posM = np.dot(Camera.A,np.dot(predict_pose,self.ones)+np.dot(np.linalg.inv(R),ts*self.sizeM))
        p1 = np.array([[posM[0,0],posM[1,0]]]).T/posM[2,0]
        p2 = np.array([[posM[0,1],posM[1,1]]]).T/posM[2,1]
        p3 = np.array([[posM[0,2],posM[1,2]]]).T/posM[2,2]
        p4 = np.array([[posM[0,3],posM[1,3]]]).T/posM[2,3]

        pure_wh = self.cat_pure_wh(p1,p2,p3,p4)

        base_wh = self.cat_base_wh(pure_wh)

        uv_vel = uv - self.pre_uv
        vel_wh = self.cat_vel_wh(uv_vel,pure_wh)


        frame =  self.cat_finallyFrame(uv,uv_vel,base_wh,vel_wh)

        #save pre
        self.pre_uv = uv
        return frame
    def cat_pure_wh(self,p1,p2,p3,p4):
        #pure_width = max([p1[0,0],p1[0,0],p1[0,0],p1[0,0]])-min([p1[0,0],p1[0,0],p1[0,0],p1[0,0]])
        #pure_height = max([p1[1,0],p1[1,0],p1[1,0],p1[1,0]])-min([p1[1,0],p1[1,0],p1[1,0],p1[1,0]])
        #pure_wh = [pure_width,pure_height]
        return np.array([max([p1[0,0],p2[0,0],p3[0,0],p4[0,0]])-min([p1[0,0],p2[0,0],p3[0,0],p4[0,0]]),\
                max([p1[1,0],p2[1,0],p3[1,0],p4[1,0]])-min([p1[1,0],p2[1,0],p3[1,0],p4[1,0]])])
    def cat_base_wh(self, pure_wh):
        #base_window_w = anzenK * pure_wh[0]
        #base_window_h = anzenK * pure_wh[1]
        #base_window_wh = anzenK*pure_wh
        return UvApriltag.anzenK*pure_wh
    def cat_vel_wh(self,uv_vel,pure_wh):
        Bwh = np.array([UvApriltag.anzenK + UvApriltag.uv_velK*abs(int(uv_vel[0,0])),\
                UvApriltag.anzenK + UvApriltag.uv_velK*abs(int(uv_vel[1,0])) ])
        return Bwh*pure_wh

    def cat_pureFrame(self,uv,pure_wh):
        uv = np.array([uv[0,0],uv[1,0]])
        lefttop = np.squeeze(uv.T - 0.5*pure_wh)
        rightbottom = np.squeeze(uv.T + 0.5*pure_wh)
        if(np.any(lefttop > rightbottom)):
            lefttop[0] = 0
            lefttop[1] = 0
            rightbottom[0]=Camera.image_size[0]
            rightbottom[1]=Camera.image_size[1]

            caution = '+' * 30 + '\n'
            caution += '+{:^28}+\n'.format('Caution')
            caution += '+' * 30 + '\n'
            colored_warning = termcolor.colored(caution, 'yellow')
            print(colored_warning)
        return np.array([lefttop,rightbottom])

    def cat_finallyFrame(self,uv,uv_vel,basis_wh,vel_wh):
        uv = np.array([uv[0,0],uv[1,0]])
        uv_vel = np.squeeze(uv_vel)
        lefttop = np.squeeze(uv.T - 0.5*vel_wh)
        rightbottom = np.squeeze(uv.T + 0.5*vel_wh)

        uv_vel[0] = int(uv_vel[0])
        uv_vel[1] = int(uv_vel[1])
        if(uv_vel[0]>0):
            lefttop[0] = uv[0]-0.5*basis_wh[0]
        elif(uv_vel[0]<0):
            rightbottom[0] = uv[0]+0.5*basis_wh[0]
        else:
            pass
        if(uv_vel[1]>0):
            lefttop[1] = uv[1]-0.5*basis_wh[1]
        elif(uv_vel[1]<0):
            rightbottom[1] = uv[1]+0.5*basis_wh[1]
        else:
            pass

        if(lefttop[0]<0):lefttop[0]=0
        if(lefttop[1]<0):lefttop[1]=0
        if(rightbottom[0]>Camera.image_size[0]):
            rightbottom[0]=Camera.image_size[0]
        if(rightbottom[1]>Camera.image_size[1]):
            rightbottom[1]=Camera.image_size[1]
        if(np.any(lefttop > rightbottom)):
            lefttop[0] = 0
            lefttop[1] = 0
            rightbottom[0]=Camera.image_size[0]
            rightbottom[1]=Camera.image_size[1]

            caution = '+' * 30 + '\n'
            caution += '+{:^28}+\n'.format('Caution')
            caution += '+' * 30 + '\n'
            colored_warning = termcolor.colored(caution, 'yellow')
            print(colored_warning)

        return np.array([lefttop,rightbottom])




        


