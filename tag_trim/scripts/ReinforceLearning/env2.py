import math
import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np

class Action():
    anzenK_UP    = 0
    anzenK_DOWN  = 1
    uv_velK_UP   = 2
    uv_velK_DOWN = 3
    K_STAY = 4
    @classmethod
    def act(cls,action):
        if(action == Action.anzenK_UP ):
            return  0.1, 0.0
        elif(action == Action.anzenK_DOWN ):
            return -0.1, 0.0
        elif(action == Action.uv_velK_UP ):
            return   0., 1.0
        elif(action == Action.uv_velK_DOWN ):
            return   0.,-1.0
        elif(action == Action.K_STAY):
            return   0., 0.
        else:
                logger.warn(
                    "You are calling 'step()' even though this "
                    "You are can't act this "
                )
                exit(-1)

class Environment2():
    def __init__(self):


        self._k1_threshold=(1.0,5.0)  # anzen
        self._k2_threshold=(0.0,10.0) # uvvel
        low = np.array([self._k1_threshold[0],
                 self._k2_threshold[0],
                 ],
                dtype=np.float32)
        high = np.array([self._k1_threshold[1],
                 self._k2_threshold[1]],
                dtype=np.float32)

        self.observation_space = spaces.Box(low,high, dtype=np.float32)
        self.action_space = spaces.Discrete(5)

        self.seed()
        self.state = None
        self.steps_beyond_done = None
    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    def step(self, states_param, action):
        err_msg = "%r (%s) invalid" % (action, type(action))
        assert self.action_space.contains(action), err_msg

        act = Action.act(action)
        info = str(self.state)+","+str(act)+","
        self.state += act
        info += str(self.state)

        k1,k2 = self.state
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
        return np.array(self.state), reward, done , info#{}
    def reset(self):
        self.state = self.np_random.uniform(low=1.01, high=5.0, size=(2,))
        self.steps_beyond_done = None
        return np.array(self.state)
    def render(self):
        raise Exception("You have to implements transform method")


if __name__ == "__main__":
    e = Environment2()
    e.reset()
    s,r,done,info = e.step((1,4),1)
