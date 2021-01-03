import time

class TCR:
    def __init__(self):
        self.init_time =\
                self.begin_time =\
                self.pre_time =\
                self.reset_time =\
                time.time()
    @property
    def now(self):
        return time.time() - self.init_time
    def reset(self):
        self.reset_time = time.time() - self.init_time
    def between(self):
        return self.now - self.reset_time
    def response(self):
        return self.begin_time - self.pre_time
    def begin(self):
        self.begin_time = time.time() - self.init_time
    def end(self):
        self.pre_time = time.time() - self.init_time

ad = TCR()
ad.between()

