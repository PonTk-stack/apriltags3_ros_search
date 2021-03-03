import numpy as np
class Action():
    anzenK_UP = "aUP"
    anzenK_DOWN = "aDOWN"
    uv_velK_UP = "veUP"
    uv_velK_DOWN = "veDOWN"




class UvApriltagState():
    def __init__(self, anzenK, uv_velK , flag=False\
            ,pure_pixel=0 ,pixel=0):
        self.detected = flag
        self.anzenK = anzenK
        self.uv_velK = uv_velK
        self.pure_pixel = pure_pixel
        self.pixel = pixel
    @property
    def s(self):
        return self.digitize_state(self.params)
    def digitize_state(self,params):
        #anzenK, uv_velK = agent_state.get_param()
        K1, K2  = params
        digitized = [np.digitize(K1 , bins = self.bins(1.0,3.0,16)),\
                     np.digitize(K2, bins=self.bins(0.0,4.0,16))]
        return sum([x*(16**i) for i,x in enumerate(digitized)])
    def bins(self, clip_min, clip_max, num):
        return np.linspace(clip_min, clip_max, num + 1)[1:-1]
    @property
    def clone(self):
        return UvApriltagState(self.anzenK, self.uv_velK\
                ,flag=self.detected,pure_pixel=self.pure_pixel\
                ,pixel=self.pixel)
    @property
    def params(self):
        return [self.anzenK, self.uv_velK]
    @property
    def params_for_reward(self):
        return [self.detected,self.pure_pixel,self.pixel]

    def get_param(self):
        return self.anzenK, self.uv_velK
    def set_for_reward(self,detect_flag,pure_pixel,pixel):
        self.detected = detect_flag
        self.pure_pixel = pure_pixel
        self.pixel = pixel
    def set_param(self,anzenK,uv_velK,detect_flag):
        self.anzenK =anzenK
        self.uv_velK =uv_velK
        self.detected = detect_flag
    def action(self,action):
        if(self.anzenK < 3. and action == Action.anzenK_UP ):self.anzenK += 0.125
        elif(self.anzenK>1. and action == Action.anzenK_DOWN ):self.anzenK -= 0.125
        elif(self.uv_velK<4. and action == Action.uv_velK_UP ):self.uv_velK += 0.25
        elif(self.uv_velK>0. and action == Action.uv_velK_DOWN ):self.uv_velK -= 0.25
        """
        else:
            print("error: UvApriltagAgent")
            exit(0)
        """
