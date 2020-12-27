#!/usr/bin/env python
#from agent import BAgent
from agent import *
from collections import defaultdict
class QLearningAgent(BAgent, object):
    def __init__(self, env, epsilon = 0.1):
        super(QLearningAgent, self).__init__(epsilon)
        self.env = env

        self.scene = 1
        #self.lapriltag_ros = LApriltag_ros()
        #self.imageconv_ros = ImageConverter_ros()
###############################################################
        self.init_learn()
        self.episode = 0
    def init_learn(self):
        self.init_log()
        self.Q = defaultdict(lambda: [0] * len(self.env.actions))
        self.reward = 0.
        self.state = self.env.reset()
    def reset_episode(self):
        self.episode +=1
        self.state = self.env.reset()
        self.scene = 1

        self.log(self.reward)
        #self.show_reward_log(episode = self.episode)
    def learn(self,detect_flag,pure_pixel, pixel, gamma=0.9, learn_rate=0.3 ):
        self.env.update_for_agent_state(detect_flag,\
                pure_pixel, pixel)
        action=self.policy(\
                self.state,self.env.actions, self.Q )
        n_state, reward, done, info = self.env.step(action)
        self.reward = reward
        print("anzenK : {}, uv_velK : {} , reward : {}".format(\
                info[0],info[1],reward))


        gain = reward + gamma * max(self.Q[n_state])
        estimated = self.Q[self.state][action]
        self.Q[self.state][action] += \
                learn_rate*(reward+gain*estimated)

        self.state = n_state
        self.scene +=1
        return info 


