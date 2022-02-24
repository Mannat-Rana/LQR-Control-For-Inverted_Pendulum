#!/usr/bin/env python3
from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import rospy
import control
import math
import numpy as np

# Got these constants from Gazebo directly
m = 2
M = 20
g = 9.8
l = 0.5

# Got the A and B matrix from class lecture slides
A = np.matrix([ [0, 1,                                       0, 0],
                [0, 0,            (-12 * m * g)/((12 * M) + m), 0],
                [0, 0,                                       0, 1],
                [0, 0, (12 * g * (M + m))/(l * ((13 * M) + m)), 0]
                ])

B = np.matrix([ [0],
                [13 / ((13 * M) + m)],
                [0],
                [-12 / (l * ((13 * M) + m))]
                ])

Q = np.diag( [1,1,1,1.] ) * 75
R = np.diag( [1.] )

# Use control module to get optimal gains
K, S, E = control.lqr( A, B, Q, R )

class LQR_InvertedPendulum_Controller:
    def __init__(self):
        rospy.init_node('LQR_Controller')
        # Publishing to joint1 command vel to move cart easier
        self.command_pub = rospy.Publisher("/invpend/joint1_velocity_controller/command",
                                            Float64, queue_size=10)
        # Subscribing to two topics to get angular pos/vel and linear pos/vel of pendulum
        self.theta_sub = rospy.Subscriber("/invpend/joint2_position_controller/state",
                                          JointControllerState, self.theta_callback)
        self.pos_sub = rospy.Subscriber("/invpend/joint_states",
                                        JointState, self.pos_callback)
        self.current_state = np.array([0., 0., 0., 0.])
        self.desired_state = np.array([0., 0., 0., 0.])
        self.command_msg = Float64()
    
    def theta_callback(self, theta_msg):
        # Callback to update angular state variables
        self.current_state[2] = theta_msg.process_value
        self.current_state[3] = theta_msg.process_value_dot
        rospy.loginfo_throttle(2, f'Current Angle: {math.degrees(theta_msg.process_value)}')
        
    def pos_callback(self, pos_msg):
        # Callback to update linear state variables
        self.current_state[0] = pos_msg.position[1]
        self.current_state[1] = pos_msg.velocity[1]
        
    def balance(self):
        # Get control output by multiplying K with state error
        self.command_msg.data = np.matmul(K, (self.desired_state - self.current_state))
        self.command_pub.publish(self.command_msg)
        rospy.loginfo_throttle(2, f'Commanding: {self.command_msg.data}')

def main():
    b = LQR_InvertedPendulum_Controller()
    while not rospy.is_shutdown():
        b.balance()          

if __name__ == '__main__':
    main()