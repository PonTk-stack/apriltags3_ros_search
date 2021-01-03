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
        self.episode = 1
        self.action = 0
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
    def learn(self,detect_flag,pure_pixel, pixel, gamma=0.9, learn_rate=0.2 ):
        self.env.update_for_agent_state(detect_flag,\
                pure_pixel, pixel)
        n_action=self.policy(\
                self.state,self.env.actions, self.Q )
        n_state, reward, done, info = self.env.step(n_action)
        self.reward = reward

        """
        if(done):
            self.reset_episode()
        gain = reward + gamma * max(self.Q[n_state])
        estimated = self.Q[self.state][self.action]
                #learn_rate*(reward+gain*estimated)
        """
        self.Q[self.state][self.action] =\
                (1-learn_rate)*self.Q[self.state][self.action] +\
                learn_rate*(reward+gamma*self.Q[n_state][n_action])
        print("anzenK:{},uv_velK:{},reward:{},detect:{},\n\
                action:{},s:{},Q:{}".format(\
            info[0],info[1],reward,detect_flag,\
            self.action,self.state,self.Q[self.state][self.action]))

        self.action = n_action
        self.state = n_state
        self.scene +=1
        return info 


