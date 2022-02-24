#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

# initialize node
rospy.init_node("reset_test")

# define service
#rospy.wait_for_service("/gazebo/reset_simulation")
reset_sim = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)

# invoke service
reset_sim()
