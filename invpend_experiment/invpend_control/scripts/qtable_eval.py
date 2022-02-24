#!/usr/bin/env python


""" Make a cart-pole system to keep balance using Q Learning """


# Import utilities
from __future__ import print_function
import numpy as np
import math
import random
import time
import matplotlib.pyplot as plt
# Import rospy for ros manipulation
import rospy
# Import CartPole class from cartpole.py
from cartpole_v0 import CartPole, bcolors
from qtable_train import observeToBucket, select_action


# Time the code execution
start_time = time.time()
# Reinforement learning environment related settings
## Discrete actions, states and buckets
ACTIONS = (-200., 0., 200.) # discrete velocity command
NUM_ACTIONS = len(ACTIONS)
upper_bound = [2.4, 1, math.pi/12, math.radians(50)]
lower_bound = [-2.4, -1, -math.pi/12, -math.radians(50)]
STATE_BOUNDS = zip(lower_bound, upper_bound)
NUM_BUCKETS = (1, 1, 6, 3) # (pos_cart, vel_cart, pos_pole, vel_pole)
## Simulation related constans
NUM_EPISODES = 10
MAX_STEP = 2500

def main():
    """ Set up Q-learning and run """
    print("Initiating simulation...")
    rospy.init_node('q_learning')
    evalip = CartPole()
    # Initialize
    step = 0
    episode = 0
    num_streaks = 0
    # load Q table
    q_table = np.load('qtable_storage/q_table18-02-07-18-09.npy')
    # reset environment
    evalip.reset_env()
    # initial joint states
    ob, _, _ = evalip.observe_env()
    # map joint states to slot in Q table
    state_0 = observeToBucket(ob)
    # make space to store episodic reward accumulation
    accumulated_reward = 0
    reward_list = []
    # get ready to test
    rate = rospy.Rate(evalip.freq)
    while not rospy.is_shutdown() and episode < NUM_EPISODES:
        print(bcolors.OKBLUE, "::: Episode {0:d}, Step {1:d}".format(episode, step), bcolors.ENDC)
        # select an action regarding to Q table
        action_index, action = select_action(q_table, state_0, explore_rate=0)
        # apply action as velocity comand
        evalip.take_action(action)
        # give the enviroment some time to obtain new observation
        rate.sleep()
        # obtain new observation from environment
        ob, reward, out = evalip.observe_env()
        # map new observation to slot in Q table
        state = observeToBucket(ob)
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
            evalip.reset_env()
            # back to initial joint states
            ob, _, _ = evalip.observe_env()
            # map joint states to slot in Q table
            state_0 = observeToBucket(ob)
            step = 0
            episode += 1
            accumulated_reward = 0

    # stop sending velocity command
    evalip.clean_shutdown()
    plt.plot(reward_list)
    plt.xlabel('Episode')
    plt.ylabel('Accumulated reward')
    plt.show()
    rospy.on_shutdown(evalip.clean_shutdown)
    rospy.spin()
    
if __name__ == '__main__':
    main()
