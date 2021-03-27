# -*- coding: utf-8 -*-
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import gym

from env2_z import Environment2_z
from dqn_agent import DQN

import os


# Hyper Parameters
#env = env.unwrapped


class LSTM_Net(nn.Module):
    def __init__(self ,N_STATES, N_ACTIONS):
        super(LSTM_Net, self).__init__()
        HIDDEN_DIM = 5
        self.rnn = nn.LSTM(input_size = N_STATES,
                hidden_size = HIDDEN_DIM,
                batch_first = True)
        self.out = nn.Linear(HIDDEN_DIM, N_ACTIONS)
    def forward(self, x, hidden0 = None):
        x,(hidden, cell) = self.rnn(x, hidden0)
        actions_value = self.out(x)
        return actions_value

class Net_ver2(nn.Module):
    def __init__(self ,N_STATES, N_ACTIONS):
        super(Net_ver2, self).__init__()
        self.fc1 = nn.Linear(N_STATES, 400)
        self.fc1.weight.data.normal_(0, 0.1)   # initialization
        self.fc2 = nn.Linear(400, 20, bias=False)
        self.fc2.weight.data.normal_(0, 0.1)   # initialization
        self.out = nn.Linear(20, N_ACTIONS)
        self.out.weight.data.normal_(0, 0.1)   # initialization
    def forward(self, x):
        x = self.fc1(x)
        x = F.relu(x)
        x = self.fc2(x)
        x = F.relu(x)
        actions_value = self.out(x)
        return actions_value


class DQNv2(DQN, object):  # add z param
    def __init__(self, observation_space, action_space):
        super(DQNv2, self).__init__(observation_space, action_space)

        self.eval_net   = Net_ver2(self._N_STATES,self._N_ACTIONS)
        self.target_net = Net_ver2(self._N_STATES,self._N_ACTIONS)

        self.learn_step_counter = 0                                     # for target updating
        self.__eval_net_save_iter = 0                                     # for target updating
        self.optimizer = torch.optim.Adam(self.eval_net.parameters(), lr=self._LR)
        self.loss_func = nn.MSELoss()



print('\nCollecting experience...')
if __name__ == "__main__":
    env = Environment2_z()
    dqn = DQNv2(env.observation_space, env.action_space)
    for i_episode in range(10000):
        s = env.reset()

        ep_r = 0
        while True:
            #env.render()

            s[2] = 2.0
            a = dqn.choose_action(s)
            # take action
            s_, r, done, info = env.step(s,a)
            # modify the reward
            k1,k2,_ = s_
            '''
            r1 = (env.x_threshold - abs(x)) / env.x_threshold - 0.8
            r2 = (env.theta_threshold_radians - abs(theta)) / env.theta_threshold_radians - 0.5
            r = r1 + r2
            '''
            dqn.store_transition(s, a, r, s_)
            ep_r += r

            if dqn.memory_counter > dqn._MEMORY_CAPACITY:
                dqn.learn()
                if done:
                    print('Ep: ', i_episode,
                          '| Ep_r: ', round(ep_r, 2))
            if done:
                break
            s = s_
