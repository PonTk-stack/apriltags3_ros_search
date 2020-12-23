class Action():
    anzenK_UP = "aUP"
    anzenK_DOWN = "aDOWN"
    uv_velK_UP = "veUP"
    uv_velK_DOWN = "veDOWN"

class UvApriltagState():
    def __init__(self, anzenK, uv_velK):
        self.detected = False
        self.anzenK = anzenK
        self.uv_velK = uv_velK
    def clone(self):
        return UvApriltagState(self.anzenK, self.uv_velK)

    def get_param(self):
        return self.anzenK, self.uv_velK
    def set_for_reward(self,detect_flag):
        self.detected = detect_flag
    def get_for_reward(self,detect_flag):
        return self.detected
    def set_param(self,anzenK,uv_velK,detect_flag):
        self.anzenK =anzenK
        self.uv_velK =uv_velK
        self.detected = detect_flag
    def action(self,action):
        if(action == Action.anzenK_UP ):self.anzenK += 0.1
        elif(action == Action.anzenK_DOWN ):self.anzenK -= 0.1
        elif(action == Action.uv_velK_UP ):self.uv_velK += 0.01
        elif(action == Action.uv_velK_DOWN ):self.uv_velK -= 0.01
        else:
            print("error: UvApriltagAgent")
            exit(0)
