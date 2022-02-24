# Big Picture
* *Initialize: Build a simulation environment with an inverted pendulum in gazebo*
  - [x] Create inverted pendulum model using URDF, and visualize the model in rviz
  - [x] Spawn the inverted pendulum in gazebo
  - [x] Move the inverted pendulum using ROS
  - [x] Generate reinforcement learning friendly environment with ROS+Gazebo combination
    - [x] Read joint states from model
    - [x] Enables reset environemnt function \(current major problem\)
    - [x] Test velocity control on cart 
    - [x] Feedback with reward for each state
  - [ ] Implement deep reinforcement learning to control the inverted pendulum
    - [ ] Q-table learning
    - [ ] Q-network learning
    - [ ] Deep Q Network
    - [ ] Deep Deterministic Policy Gradient
* *Build simulation environment with model of the robot in gazebo*
  - [ ] Create Kuka's model using URDF \(partly available at [kuka_experiment](https://github.com/ros-industrial/kuka_experimental/tree/indigo-devel/kuka_kr10_support)\)
  - [ ] Combine simulated robot model with real video data
  - [ ] Use Kinect\(or reasonable alternative\) to record objects' moving trajectory.
* *Simulated robot learns to catch with real human data feed*
* *Ultimate goal: The robot catches the ball thrown by human*