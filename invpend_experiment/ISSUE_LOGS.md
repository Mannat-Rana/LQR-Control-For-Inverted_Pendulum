# Current Issues:
- \[Fixed\] It seems ros_control was keeping exerting control to the joints, which maintains the pole not falling down.
  - **Set pid to 0, 0, 0 in config yaml file, to disable position control**
- \[Fixed\] Cannot reset simulation: if "/gazebo/reset_simulation" service called, gazebo model went back to initial, however "ROS time moved backwards" error appeared and cannot get data from all topics. Logs are as follows. _In `invpend_control/launch/load_invpend.launch`, line 4, set ros parameter `use_sim_time` to `false` or in complete `<arg name="use_sim_time" default="false"/>`. But, this will cause reset time consumed to be accumulated due to the mechanism in `/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/timer.py`_
  - **Publish `/gazebo/set_link_state` topic to reset, instead of using `/gazebo/reset_simulation` service. First, set pole link w.r.t. cart link, then set cart link w.r.t. slidebar link**

>
```
Traceback (most recent call last):
  File "invpend_control/scripts/vel_ctrl_test.py", line 86, in <module>
    main()
  File "invpend_control/scripts/vel_ctrl_test.py", line 82, in main
    cart.wobble()
  File "invpend_control/scripts/vel_ctrl_test.py", line 59, in wobble
    rate.sleep()
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/timer.py", line 103, in sleep
    sleep(self._remaining(curr_time))
  File "/opt/ros/kinetic/lib/python2.7/dist-packages/rospy/timer.py", line 164, in sleep
    raise rospy.exceptions.ROSTimeMovedBackwardsException(time_jump)
rospy.exceptions.ROSTimeMovedBackwardsException: ROS time moved backwards
Shuting dwon...
```

- Have to run script immediately after `launch invpend_control load_invpend.launch`, or `reset_env` brings the pole angle to 2*pi, which is totally OK in the real world but may disarm my script's judgement of pole angle out of range.
- Using settings of [CartPole-v0](https://github.com/openai/gym/blob/master/gym/envs/classic_control/cartpole.py) in openAI gym, mass of cart and pole too small results in slow motion in gazebo simulation.
