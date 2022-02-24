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
import rospy
import tensorflow as tf
# Import CartPole class from cartpole.py
from cartpole_v1 import CartPole, bcolors
# Import debug tool
import pdb


# Time the code execution
start_time = time.time()
# Reinforement learning environment related settings
## Discrete actions
ACTIONS = (-5., -4., -3., -2., -1., 0., 1., 2., 3., 4., 5.) # discrete velocity command
NUM_ACTIONS = len(ACTIONS)
## Neural Network parameters
INPUT_LAYER_SIZE = 4
HIDDEN_LAYER_SIZE = 128
OUTPUT_LAYER_SIZE = NUM_ACTIONS
## Simulation related constans
NUM_EPOCHS = 1000
MAX_STEP = 250
STREAK_TO_END = 120
MIN_EXPLORE_RATE = 0.05

class QlearnCartPole(CartPole):
    """ Inherent from CartPole class and add q-learning method """
    def __init__(self):
        CartPole.__init__(self)

    def train(self):
        # Traing settings
        gamma = .99 # discount_factor
        epsilon = get_explore_rate(0) # explore_rate
        # make space to store episodic reward accumulation
        reward_list = []
        stepcost_list = []
        # neural network settings
        ## The variables below hold all the trainable weights.
        ## also try 1 layer model
        states_node = tf.placeholder(dtype=tf.float32, shape=(1,INPUT_LAYER_SIZE))
        qvalue_node = tf.placeholder(dtype=tf.float32, shape=(1,OUTPUT_LAYER_SIZE))
        w1 = tf.Variable(tf.truncated_normal(shape=[INPUT_LAYER_SIZE, HIDDEN_LAYER_SIZE],
                                             stddev = 0.01, dtype=tf.float32))
        b1 = tf.Variable(tf.constant(value=0.01, shape=[HIDDEN_LAYER_SIZE], dtype=tf.float32))
        w2 = tf.Variable(tf.truncated_normal(shape=[HIDDEN_LAYER_SIZE, OUTPUT_LAYER_SIZE],
                                             stddev = 0.01, dtype=tf.float32))
        b2 = tf.Variable(tf.constant(value=0.01, shape=[OUTPUT_LAYER_SIZE], dtype=tf.float32))
        ## define the neural network
        def model(input_states):
            hidden = tf.nn.relu(tf.matmul(input_states, w1) + b1)
            return tf.matmul(hidden, w2) + b2
        ## output
        q_values = model(states_node)
        ## predictions, also try tf.nn.softmax(q_values)
        prediction = tf.argmax(q_values, axis=1)
        # Loss function, also try tf.reduce_sum(tf.square(qvalue_node - q_values))
        # loss = tf.reduce_mean(tf.nn.sparse_softmax_cross_entropy_with_logits(
        #     labels=qvalue_node, logits=q_values))
        loss = tf.reduce_sum(tf.square(qvalue_node - q_values))
        # L2 regularization for the fully connected parameters.
        regularizers = (tf.nn.l2_loss(w1) + tf.nn.l2_loss(b1) +
                        tf.nn.l2_loss(w2) + tf.nn.l2_loss(b2))
        ## (optional) add the regularization term to the loss, uncomment following line.
        # loss += 5e-4 * regularizers
        ## (optional) learning rate decaying, uncomment following lines
        # learning_rate = tf.train.exponential_decay(
        #     0.1,                # Base learning rate.
        #     batch * BATCH_SIZE,  # Current index into the dataset.
        #     train_size,          # Decay step.
        #     0.95,                # Decay rate.
        #     staircase=True)
        ## optimizer, try MomentumOptimizer, AdamOptimizer, etc
        optimizer = tf.train.GradientDescentOptimizer(learning_rate=0.1).minimize(loss)
        
        with tf.Session() as sess:
            sess.run(tf.global_variables_initializer())
            rate = rospy.Rate(self.freq)
            for epoch in range(NUM_EPOCHS):
                # reset environment
                self.reset_env()
                step = 0
                accumulated_reward = 0
                out = False
                while step < MAX_STEP and not out and not rospy.is_shutdown():
                    print(bcolors.OKBLUE, "::: Epoch {0:d}, Step {1:d}, Time Elapsed: {2:.7f}".format(epoch, step, self.time_elapse), bcolors.ENDC)
                    # initial joint states
                    ob, _, out = self.observe_env()
                    q, argmax_q = sess.run([q_values, prediction],
                                               feed_dict={states_node:np.array(ob).reshape(1,4)})
                    action_index = argmax_q[0]
                    # pick an action randomly by exploring
                    if np.random.rand(1)[0] < epsilon:
                        action_index = random.randrange(0,NUM_ACTIONS)
                        print(bcolors.WARNING, "!!! Action selected randomly !!!", bcolors.ENDC)
                    #pdb.set_trace()
                    action = ACTIONS[action_index]
                    # apply action
                    self.take_action(action)
                    # give environment some time to obtain new observation
                    rate.sleep()
                    # obtain new observation from environment
                    ob, reward, out = self.observe_env()
                    q_next = sess.run(q_values, feed_dict={states_node:np.array(ob).reshape(1,4)})
                    max_q_next = np.max(q_next)
                    target_q = q
                    target_q[0, action_index] = reward + gamma * max_q_next
                    # train network using target_q and q, remember loss = square(target_q-q)
                    opt = sess.run(optimizer,
                                   feed_dict={states_node:np.array(ob).reshape(1,4),
                                              qvalue_node:target_q})
                    accumulated_reward += reward
                    step += 1
                # learning rate decay    
                epsilon = get_explore_rate(epoch)
                # store reward and step cost
                reward_list.append(accumulated_reward)
                print("$$$ accumulated reward in epoch {0:d}: {1:.5f}".format(epoch,accumulated_reward))
                stepcost_list.append(step)
            # stop sending velocity command
            self.clean_shutdown()
            # save reward list and Q table
            reward_list = np.asarray(reward_list) # convert list to numpy array
            np.save('qnet_storage/reward_list' + datetime.datetime.now().strftime("%y-%m-%d-%H-%M") + '.npy', reward_list)
            np.save('qnet_storage/q_net' + datetime.datetime.now().strftime("%y-%m-%d-%H-%M") + '.npy', q)
            end_time = time.time() # time stamp end of code
            print(bcolors.WARNING, "@@@ Training finished...\nTraining time was {:.5f}".format(end_time - start_time), bcolors.ENDC)
            plt.plot(reward_list)
            plt.xlabel('Episode')
            plt.ylabel('Accumulated reward')
            plt.show()
            self.clean_shutdown()
        sess.close()

def get_explore_rate(episode):
    return max(MIN_EXPLORE_RATE, min(1, 1.0-math.log10((episode+1)/25.)))

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
