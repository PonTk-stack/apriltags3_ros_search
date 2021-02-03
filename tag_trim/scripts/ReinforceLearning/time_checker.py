import time

class TCR:
    def __init__(self):
        self.init_time =\
                self.__check_time =\
                self.__pre_time =\
                self.__reset_time =\
                time.time()

    
    def now(self):
        return time.time() - self.init_time
    def reset(self):
        self.__reset_time = time.time() - self.init_time
    def between(self):
        return self.now() - self.__reset_time
    def response(self):
        self.__pre_time = self.__check_time
        self.__check_time = self.now()
        return self.__check_time - self.__pre_time

ad = TCR()
ad.now()
ad.between()

