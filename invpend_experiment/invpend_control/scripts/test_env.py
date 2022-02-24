#! /usr/bin/env python
import rospy
import random
import math

from cartpole_v0 import CartPole

class Testbed(CartPole):
    def __init__(self):
        CartPole.__init__(self)
        self.start = rospy.Time.now()

    def random_move(self):
        """ Control cart with random velocity command """
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            ob, reward, out = self.observe_env()
            if self.out_range:
                self.reset_env()
            else:
                print("=== Within range, exert random vel command ===")
                vel_cmd = random.uniform(-10, 10)
                self.take_action(vel_cmd)
            rate.sleep()

    def sin_move(self):
        """ Control cart with sinusoidal velocity command """
        rate = rospy.Rate(self.freq)
        period_factor = 1
        amplitude_factor = 25
        while not rospy.is_shutdown():
            ob, reward, out = self.observe_env()
            if self.out_range:
                self.reset_env()
            else:
                print("=== Within range, exert sinusoidal vel command ===")
                elapsed = rospy.Time.now() - self.start
                w = period_factor * elapsed.to_sec()
                vel_cmd = amplitude_factor * math.cos(w*2*math.pi)
                self.take_action(vel_cmd)
            rate.sleep()

def main():
    print("Initializing node... ")
    rospy.init_node('cart_random_move')
    test_agent = Testbed()
    rospy.on_shutdown(test_agent.clean_shutdown)
    # test_agent.random_move()
    test_agent.random_move()
    rospy.spin()

if __name__ == '__main__':
    main()
