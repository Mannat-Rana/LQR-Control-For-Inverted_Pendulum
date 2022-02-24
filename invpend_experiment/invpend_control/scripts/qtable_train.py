#!/usr/bin/env python


""" Train inverted pendulum to keep balance using Q Learning """


# Import utilities
from __future__ import print_function
import numpy as np
import math
import random
import time
import datetime
import matplotlib.pyplot as plt
# Import rospy for ros manipulation
import rospy
# Import CartPole class from cartpole.py
from cartpole_v0 import CartPole, bcolors


# Time the code execution
start_time = time.time()
# Reinforement learning environment related settings
## Discrete actions, states and buckets
ACTIONS = (-200, 0, 200) # discrete force command
NUM_ACTIONS = len(ACTIONS)
upper_bound = [2.4, 1, math.pi/12, math.radians(50)]
lower_bound = [-2.4, -1, -math.pi/12, -math.radians(50)]
STATE_BOUNDS = zip(lower_bound, upper_bound)
NUM_BUCKETS = (1, 1, 6, 3) # (pos_cart, vel_cart, pos_pole, vel_pole)
## Learning related constants
MIN_LEARNING_RATE = 0.1
MIN_EXPLORE_RATE = 0.01
## Simulation related constans
NUM_EPISODES = 1000
MAX_STEP = 250
STREAK_TO_END = 120

class QlearnCartPole(CartPole):
    """ Inherent from CartPole class and add q-learning method """
    def __init__(self):
        CartPole.__init__(self)

    def train(self):
        # Initialize learning
        step = 0
        episode = 0
        learning_rate = get_learning_rate(episode)
        explore_rate = get_explore_rate(episode)
        discount_factor = 0.99
        num_streaks = 0
        # Initialize Q table
        q_table = np.zeros(NUM_BUCKETS + (NUM_ACTIONS,))
        # reset environment
        self.reset_env()
        # initial joint states
        ob, _, _ = self.observe_env()
        # map joint states to slot in Q table
        state_0 = observeToBucket(ob)
        # make space to store episodic reward accumulation
        accumulated_reward = 0
        reward_list = []
        # get ready to learn
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            print(bcolors.OKBLUE, "::: Episode {0:d}, Step {1:d}".format(episode, step), bcolors.ENDC)
            # select an action with epsilon greedy, decaying explore_rate
            action_index, action = select_action(q_table, state_0, explore_rate)
            # apply action as velocity comand
            self.take_action(action)
            # give the enviroment some time to obtain new observation
            rate.sleep()
            # obtain new observation from environment
            ob, reward, out = self.observe_env()
            # map new observation to slot in Q table
            state = observeToBucket(ob)
            # update Q-table
            print(bcolors.OKGREEN, "Q table gets update...", bcolors.ENDC)
            max_q = np.amax(q_table[state])
            q_table[state_0 + (action_index,)] += learning_rate*(reward + discount_factor*max_q - q_table[state_0 + (action_index,)])
            if episode <= NUM_EPISODES and num_streaks < STREAK_TO_END:
                if not out and step <= MAX_STEP:
                    state_0 = state
                    accumulated_reward += 1
                    print("$$$ Accumulated reward in episode {0:d} was {1:d}".format(episode, accumulated_reward))
                    step += 1
                else:
                    if step == MAX_STEP:
                        num_streaks += 1
                    else:
                        num_streaks = 0
                    accumulated_reward += 1
                    print("$$$ Accumulated reward in episode {0:d} was {1:d}".format(episode, accumulated_reward))
                    reward_list.append(accumulated_reward)
                    # reset env for next episode
                    self.reset_env()
                    # back to initial joint states
                    ob, _, _ = self.observe_env()
                    # map joint states to slot in Q table
                    state_0 = observeToBucket(ob)
                    step = 0
                    episode += 1
                    explore_rate = get_explore_rate(episode)
                    learning_rate = get_learning_rate(episode)
                    accumulated_reward = 0
            else:
                # stop sending velocity command
                self.clean_shutdown()
                # save reward list and Q table
                reward_list = np.asarray(reward_list) # convert list to numpy array
                np.save('qtable_storage/reward_list' + datetime.datetime.now().strftime("%y-%m-%d-%H-%M") + '.npy', reward_list)
                np.save('qtable_storage/q_table' + datetime.datetime.now().strftime("%y-%m-%d-%H-%M") + '.npy', q_table)
                end_time = time.time() # time stamp end of code
                print(bcolors.WARNING, "@@@ Training finished...\nTraining time was {:.5f}".format(end_time - start_time), bcolors.ENDC)
                plt.plot(reward_list)
                plt.xlabel('Episode')
                plt.ylabel('Accumulated reward')
                plt.show()
                break
            rate.sleep()

# Useful functions
def get_learning_rate(episode):
    return max(MIN_LEARNING_RATE, min(0.5, 1.0-math.log10((episode+1)/25.)))

def get_explore_rate(episode):
    return max(MIN_EXPLORE_RATE, min(1, 1.0-math.log10((episode+1)/25.)))
    
def observeToBucket(state):
    bucket_indice = []
    for i in range(len(state)):
        if state[i] <= STATE_BOUNDS[i][0]:
            bucket_index = 0
        elif state[i] >= STATE_BOUNDS[i][1]:
            bucket_index = NUM_BUCKETS[i] - 1
        else:
            # Mapping the state bounds to the bucket array
            bound_width = STATE_BOUNDS[i][1] - STATE_BOUNDS[i][0]
            offset = (NUM_BUCKETS[i]-1)*STATE_BOUNDS[i][0]/bound_width
            scaling = (NUM_BUCKETS[i]-1)/bound_width
            bucket_index = int(round(scaling*state[i] - offset))
        bucket_indice.append(bucket_index)
    return tuple(bucket_indice)

def select_action(q_table, state, explore_rate):
    # Select a random action
    if random.random() < explore_rate:
        act_idx = random.randrange(0,NUM_ACTIONS)
        action = ACTIONS[act_idx]
        print("!!! Action selected randomly !!!")
    # Select the action with the highest q
    else:
        act_idx = np.argmax(q_table[state])
        action = ACTIONS[act_idx]
        print("||| Action selected greedily |||")
    return act_idx, action

def main():
    """ Set up Q-learning and run """
    print("Initiating simulation...")
    rospy.init_node('q_learning')
    ql_agent = QlearnCartPole()
    rospy.on_shutdown(ql_agent.clean_shutdown)
    ql_agent.train()
    rospy.spin()
    
if __name__ == '__main__':
    main()
