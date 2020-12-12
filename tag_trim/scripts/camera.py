from sensor_msgs.msg import Image, CameraInfo
import numpy as  np

class Camera(object):
    A = np.array([  [0,0,0],\
                    [0,0,0],\
                    [0,0,0] ])
    K = np.array([  [0,0,0],\
                    [0,0,0],\
                    [0,0,0] ])
    image_size = [1280, 720]
    def __init__(self):

        self.A = np.array([[0,0,0],\
                    [0,0,0]])
        self.K = np.array([[0,0,0],\
                    [0,0,0]])
    @classmethod
    def setICP(cls,camera_info):
        kx = camera_info.K[0]
        ox = camera_info.K[2]
        ky = camera_info.K[4]
        oy = camera_info.K[5]

        fx = camera_info.P[0]
        cx = camera_info.P[2]
        fy = camera_info.P[5]
        cy = camera_info.P[6]

        k1 = camera_info.D[0]
        k2 = camera_info.D[1]
        p1 = camera_info.D[2]
        p2 = camera_info.D[3]
        k3 = camera_info.D[4]

        Camera.K = np.array([\
                        [kx,0.0,ox],\
                        [0.0,ky,oy],\
                        [0.0,0.0,1.0]\
                                ])
        Camera.A = np.array([\
                        [fx,0.0,cx],\
                        [0.0,fy,cy],\
                        [0.0,0.0,1.0]\
                                ])

