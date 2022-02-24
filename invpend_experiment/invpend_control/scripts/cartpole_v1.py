#!/usr/bin/env python


""" Configure a cart-pole system spawned in Gazebo to be a qualified environment for reinforcement learning 
    Based on cartpole-v0, but increases pole swaying angle limit and modifies reward mechanism"""


from __future__ import print_function

import numpy as np
import math
import random
import time

import rospy
from std_msgs.msg import (UInt16, Float64)
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty
from gazebo_msgs.msg import LinkState
from geometry_msgs.msg import Point

def exceedRange(pos_cart, pos_pole):
    return math.fabs(pos_cart) > 2.4 or math.fabs(pos_pole) > math.pi/4 # cart: +-2.4; pole: +-45degrees

class bcolors:
    """ For the purpose of print in terminal with colors """
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'
    
class CartPole(object):
    """ A cart-pole on a sliding bar;
    only the cart can be actuated;
    """
    def __init__(self):
        # init topics and services
        self._sub_invpend_states = rospy.Subscriber('/invpend/joint_states', JointState, self.jstates_callback)
        self._pub_vel_cmd = rospy.Publisher('/invpend/joint1_velocity_controller/command', Float64, queue_size=50)
        self._pub_set_pole = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=50)
        self._pub_set_cart = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=50)
        # init observation parameters
        self.pos_cart = 0
        self.vel_cart = 0
        self.pos_pole = 0
        self.vel_pole = 0
        self.reward = 0
        self.out_range = False
        self.reset_stamp = time.time()
        self.time_elapse = 0.
        # init reset_env parameters
        self.reset_dur = .2 # reset duration, sec
        self.freq = 50 # topics pub and sub frequency, Hz
        ## pole
        self.PoleState = LinkState()
        self.PoleState.link_name = 'pole'
        self.PoleState.pose.position = Point(0.0, -0.275, 0.0) # pole's position w.r.t. world
        self.PoleState.reference_frame = 'cart'
        ## cart
        self.CartState = LinkState()
        self.CartState.link_name = 'cart'
        self.CartState.pose.position = Point(0.0, 0.0, 0.0) # pole's position w.r.t. world
        self.CartState.reference_frame = 'slidebar'
        # velocity control command
        self.cmd = 0        

    def jstates_callback(self, data):
        """ Callback function for subscribing /invpend/joint_states topic """
    	# rospy.loginfo("~~~Getting Inverted pendulum joint states~~~")
    	self.pos_cart = data.position[1]
    	self.vel_cart = data.velocity[1]
    	self.pos_pole = data.position[0]
    	self.vel_pole = data.velocity[0]
        self.out_range = exceedRange(self.pos_cart, self.pos_pole)
        self.time_elapse = time.time() - self.reset_stamp
        if self.out_range == True:
            self.reward = 0.
        else:
            self.reward = 0 - math.fabs(self.pos_pole)

    def reset_env(self):
        """ Reset cart-pole to initial state"""
        rate = rospy.Rate(self.freq)
        reset_count = 0
        print(bcolors.WARNING, "\n=== Reset cart-pole to initial ===\n", bcolors.ENDC)
        while not rospy.is_shutdown() and reset_count < self.reset_dur*self.freq:
            # print("=reset counter: ", str(reset_count)) # debug log
            self._pub_vel_cmd.publish(0)
            self._pub_set_pole.publish(self.PoleState)
            self.pos_cart = 0
            self.vel_cart = 0
            self.pos_pole = 0
            self.vel_pole = 0
            self.reward = 0.
            # self.out_range = False
            reset_count += 1
            rate.sleep()
        rate = rospy.Rate(self.freq)
        reset_count = 0
        print(bcolors.WARNING, "\n=== Reset cart-pole ===\n", bcolors.ENDC)
        while not rospy.is_shutdown() and reset_count < self.reset_dur*self.freq:
            # print("=reset counter: ", str(reset_count)) # debug log
            self._pub_vel_cmd.publish(0)
            self._pub_set_cart.publish(self.CartState)
            self._pub_set_pole.publish(self.PoleState)
            self.pos_cart = 0
            self.vel_cart = 0
            self.pos_pole = 0
            self.vel_pole = 0
            self.reward = 0
            # self.out_range = False
            reset_count += 1
            rate.sleep()
        self.reset_stamp = time.time()
                
    def observe_env(self):
        """ Get cart-pole state, reward and out of range flag from environment """
        # For debug purpose, uncomment the following line
        print("~~~ Observation: cart_position: {0:.5f}, cart_velocity: {1:.5f}, pole_angle: {2:.5f}, pole_angular_velocity: {3:.5f}\nreward: {4:.5f}\nout of range: {5:} ".format(self.pos_cart, self.vel_cart, self.pos_pole, self.vel_pole, self.reward, self.out_range))
        return np.array([self.pos_cart, self.vel_cart, self.pos_pole, self.vel_pole]), self.reward, self.out_range

    def take_action(self, vel_cmd):
        self._pub_vel_cmd.publish(vel_cmd)
        print(bcolors.OKGREEN, "---> Velocity command to cart: {:.4f} m/s".format(vel_cmd), bcolors.ENDC)
        
    def clean_shutdown(self):
        print(bcolors.HEADER, "Shuting down...", bcolors.ENDC)
        self._pub_vel_cmd.publish(0)
        return True
