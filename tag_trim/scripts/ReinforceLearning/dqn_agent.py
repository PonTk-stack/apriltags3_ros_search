# -*- coding: utf-8 -*-
import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import gym

from env2 import Environment2
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
        print(x)
        actions_value = self.out(x)
        return actions_value

class Net(nn.Module):
    def __init__(self ,N_STATES, N_ACTIONS):
        super(Net, self).__init__()
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


class DQN(object):
    def __init__(self, observation_space, action_space):

        self._BATCH_SIZE = 32
        self._LR = 0.01                   # learning rate
        self._EPSILON = 0.9               # greedy policy
        self._MEMORY_CAPACITY = 100

        self._N_STATES =  observation_space.shape[0] #env.observation_space.shape[0]
        self._N_ACTIONS = action_space.n #env.action_space.n
        self._ENV_A_SHAPE = 0 if isinstance(action_space.sample(), int) else action_space.sample().shape     # to confirm the shape

        self.eval_net   = Net(self._N_STATES,self._N_ACTIONS)
        self.target_net = Net(self._N_STATES,self._N_ACTIONS)

        self.learn_step_counter = 0                                     # for target updating
        self.__eval_net_save_iter = 0                                     # for target updating
        self.memory_counter = 0                                         # for storing memory
        self.memory = np.zeros((self._MEMORY_CAPACITY, self._N_STATES * 2 + 2))     # initialize memory
        self.optimizer = torch.optim.Adam(self.eval_net.parameters(), lr=self._LR)
        self.loss_func = nn.MSELoss()

    def choose_action(self, x):
        x = torch.unsqueeze(torch.FloatTensor(x), 0)
        # input only one sample
        if np.random.uniform() < self._EPSILON:   # greedy
            actions_value = self.eval_net.forward(x)
            action = torch.max(actions_value, 1)[1].data.numpy()
            action = action[0] if self._ENV_A_SHAPE == 0 else action.reshape(self._ENV_A_SHAPE)  # return the argmax index
        else:   # random
            action = np.random.randint(0, self._N_ACTIONS)
            action = action if self._ENV_A_SHAPE == 0 else action.reshape(self._ENV_A_SHAPE)
        return action

    def store_transition(self, s, a, r, s_):
        transition = np.hstack((s, [a, r], s_))
        # replace the old memory with new memory
        index = self.memory_counter % self._MEMORY_CAPACITY
        self.memory[index, :] = transition
        self.memory_counter += 1

    def learn(self,GAMMA=0.9,TARGET_REPLACE_ITER = 100, EVAL_NET_SAVE_ITER = 100 ):

        #TARGET_REPLACE_ITER : target update frequency

        # target parameter update 
        if self.learn_step_counter % TARGET_REPLACE_ITER == 0:
            self.target_net.load_state_dict(self.eval_net.state_dict())

            #eval_net save

            if self.learn_step_counter == EVAL_NET_SAVE_ITER*self.__eval_net_save_iter:
                os.makedirs(os.environ["HOME"]+"/dqn_agent_nets", exist_ok=True)
                torch.save(self.eval_net.state_dict()\
                        ,os.environ["HOME"]+"/dqn_agent_nets"+"/eval_net"\
                                + str(self.learn_step_counter)+".pkl" )

                if self.__eval_net_save_iter >= 100:
                    self.__eval_net_save_iter+=100                                 # for target updating
                else:
                    self.__eval_net_save_iter+=10                                    # for target updating


        self.learn_step_counter += 1



        # sample batch transitions
        sample_index = np.random.choice(self._MEMORY_CAPACITY, self._BATCH_SIZE)
        b_memory = self.memory[sample_index, :]
        b_s = torch.FloatTensor(b_memory[:, :self._N_STATES])
        b_a = torch.LongTensor(b_memory[:, self._N_STATES:self._N_STATES+1].astype(int))
        b_r = torch.FloatTensor(b_memory[:, self._N_STATES+1:self._N_STATES+2])
        b_s_ = torch.FloatTensor(b_memory[:, -self._N_STATES:])

        # q_eval w.r.t the action in experience
        q_eval = self.eval_net(b_s).gather(1, b_a)  # shape (batch, 1)
        q_next = self.target_net(b_s_).detach()     # detach from graph, don't backpropagate
        q_target = b_r + GAMMA * q_next.max(1)[0].view(self._BATCH_SIZE, 1)   # shape (batch, 1)
        loss = self.loss_func(q_eval, q_target)


        #optimizerに使われていた勾配の初期化
        self.optimizer.zero_grad()
        #requires_grad=Trueとした変数に対して微分を行った勾配を計算
        loss.backward()
        #重みの更新:学習率と最適化手法に基づいて重みを更新
        self.optimizer.step()
        return loss.item()


print('\nCollecting experience...')
if __name__ == "__main__":
    env = Environment2()
    dqn = DQN(env.observation_space, env.action_space)
    for i_episode in range(10000):
        s = env.reset()
        ep_r = 0
        while True:
            #env.render()
            a = dqn.choose_action(s)
            # take action
            s_, r, done, info = env.step(s,a)
            # modify the reward
            k1,k2 = s_
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
