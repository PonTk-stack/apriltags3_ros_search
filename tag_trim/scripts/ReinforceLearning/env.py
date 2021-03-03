import numpy as np
from state import *
from collections import defaultdict
import termcolor


class Environment():
    def __init__(self, anzenK, uv_velK, default_prob = 0.8 ,):
        self.default_reward = -0.06
        self.default_prob = default_prob

        self.agent_state = UvApriltagState(anzenK, uv_velK)
        #self.init_params = [anzenK, uv_velK]
        self.init_params = self.agent_state.params

        self.pre_reward = 0
    def update_for_agent_state(self,detect_flag\
            ,pure_pixel, pixel):
        self.agent_state.set_for_reward(detect_flag\
                ,pure_pixel,pixel)

    def reset(self):
        self.agent_state = UvApriltagState(self.init_params[0], self.init_params[1])
        return self.agent_state.s
    """
    def save(self,robot):
        self.frame_init_save = self.clone(frame)
    def reset(self):
        self.count=0
        return copy.deepcopy(self.robot_init_save)
    def clone(self,obj):
        return copy.deepcopy(obj)
    """
    @property
    def actions(self):
        return[Action.anzenK_UP, Action.anzenK_DOWN, \
                Action.uv_velK_UP, Action.uv_velK_DOWN]
    def step(self,action):
        #next_state,reward,done = self.transit(self.agent_state,action)
        next_state,reward,done = self._new_transit(self.agent_state,action)

        #if next_state is not None:
        #    print("next_state is None")
        #    self.agent_state = next_state
        #self.agent_state.action()
        #n_state = self.digitize_state(next_state)
        self.agent_state =next_state
        return next_state.s,reward,done,next_state.params
    def _new_transit(self,agent_state, action):
        #next_state = agent_state.clone
        next_state = self._move(self.actions[action])
        reward,done = self._reward_func(next_state)
        return next_state,reward,done

    def transit(self,agent_state,action):
        next_states, transition_probs = self.transit_func(agent_state,action)
        if len(transition_probs)  ==0:
            print("transition_probs is None")
            return None,None,True
        #probs=[]
        #for tp in transition_probs:
        #    next_states.append(tp)
        #    probs.append(transition_probs[tp])
        next_state=np.random.choice(next_states,p=transition_probs)
        reward,done = self.reward_func(next_state)
        return next_state,reward,done

    def transit_func(self,agent_state,action):
        action = self.actions[action]
        transition_probs = []
        next_states = []
        for i,a in enumerate(self.actions):
            prob = 0.
            if action == a:
                    prob = self.default_prob
            else:
                    prob = (1.-self.default_prob)/(len(self.actions)-1)

            next_state = self._move(agent_state,a)
            next_states.append(next_state)
            transition_probs.append(prob)
        return next_states,transition_probs
    def _move(self,agent_state,action):
        next_state = agent_state
        next_state.action(action)
        return next_state
    def _reward_func(self,agent_params,config=83):
        reward = self.default_reward
        done = False
        attribute = agent_params

        if(attribute[0]==True):
            reward =(attribute[1]/attribute[2])
            self.pre_reward = reward
            done  = False 
            #done  = True
            if(agent_params.anzenK<1.0 or agent_params.uv_velK <0.0):
                reward -= 100
            return reward,done
        elif(attribute[0]==False):
            print(termcolor.colored("nondetected",'red'))
            reward = -1
            self.pre_reward = 0
            done  = True
            if(agent_params.anzenK<1.0 or agent_params.uv_velK <0.0):
                reward -= 100
            return reward,done
        else:
            #reward -= (self.count*0.001)
            return reward,done

