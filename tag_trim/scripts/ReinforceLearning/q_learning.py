#!/usr/bin/env python
from agent import BAgent
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
    def init_learn(self):
        self.init_log()
        self.Q = defaultdict(lambda: [0] * len(self.env.actions))
        self.episode = 1
        self.reward = 0.
        self.state = self.env.reset()
    def reset_episode(self):
        self.episode +=1
        self.state = self.env.reset()
        self.scene = 1

        self.log(self.reward)
        self.show_reward_log(self.episode)
    def learn(self,detect_flag, gamma=0.9, learn_rate=0.1 ):
        self.env.update_for_agent_state(detect_flag)
        action=self.policy(\
                self.state,self.env.actions )
        n_state, reward, done = self.env.step(action)
        self.reward = reward

        gain = reward + gamma * max(self.Q[n_state])
        estimated = self.Q[self.state][action]
        self.Q[self.state][action] += \
                learn_rate*(reward+gain*estimated)

        self.state = n_state
        self.scene +=1
        return n_state.get_param()


