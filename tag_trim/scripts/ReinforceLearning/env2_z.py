import math
import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np

from env2 import Environment2,Action


class Environment2_z(Environment2,object):
    def __init__(self):
        super(Environment2_z,self).__init__()
        self._k1_threshold=(1.0,5.0)  # anzen
        self._k2_threshold=(0.0,10.0) # uvvel
        low = np.array([self._k1_threshold[0],
                 self._k2_threshold[0],
                 np.finfo(np.float32).min],
                dtype=np.float32)
        high = np.array([self._k1_threshold[1],
                 self._k2_threshold[1],
                 np.finfo(np.float32).max],
                dtype=np.float32)
        self.observation_space = spaces.Box(low,high, dtype=np.float32)

    def step(self, states_param, action):
        err_msg = "%r (%s) invalid" % (action, type(action))
        assert self.action_space.contains(action), err_msg

        act = Action.act(action)


        info = str(self.state)+","+str(act)+","
        for i,a in enumerate(act):
            self.state[i] += a
        info += str(self.state)

        k1,k2,_ = self.state
        done = bool(
                k1 < self._k1_threshold[0]
                or k1 > self._k1_threshold[1]
                or k2 < self._k2_threshold[0]
                or k2 > self._k2_threshold[1]
                )
        if not done:
            reward = 1.0
        elif self.steps_beyond_done is None:
            self.steps_beyond_done = 0
            reward = 1.0
        else:
            if self.steps_beyond_done == 0:
                logger.warn(
                    "You are calling 'step()' even though this "
                    "environment2 has already returned done = True. You "
                    "should always call 'reset()' once you receive 'done = "
                    "True' -- any further steps are undefined behavior."
                )
            self.steps_beyond_done += 1
            reward = 0.0
        return np.array(self.state,dtype=np.float32), reward, done , info#{}

    def reset(self):
        self.state =np.append(self.np_random.uniform(low=1.01, high=5.0, size=(2,)) ,None  )
        self.steps_beyond_done = None
        return np.array(self.state,dtype=np.float32)




if __name__ == "__main__":
    e = Environment2_z()
    s = e.reset()
    print(s)
    s,r,done,info = e.step(s,1)
    print(s)
