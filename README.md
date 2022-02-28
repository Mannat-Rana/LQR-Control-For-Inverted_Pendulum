# LQR Control Application for an Inverted Pendulum ![SHIELD](https://img.shields.io/badge/Project%20Status%3A-Complete-green?style=for-the-badge) ![ros](https://camo.githubusercontent.com/4c117e738ecff5825b1031d601ac04bc70cc817805ba6ce936c0c556ba8e14f0/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d524f5326636f6c6f723d323233313445266c6f676f3d524f53266c6f676f436f6c6f723d464646464646266c6162656c3d) ![PYTHON](https://camo.githubusercontent.com/3df944c2b99f86f1361df72285183e890f11c52d36dfcd3c2844c6823c823fc1/68747470733a2f2f696d672e736869656c64732e696f2f7374617469632f76313f7374796c653d666f722d7468652d6261646765266d6573736167653d507974686f6e26636f6c6f723d333737364142266c6f676f3d507974686f6e266c6f676f436f6c6f723d464646464646266c6162656c3d) 

## About
The goal of this application is to develop a LQR based controller to balance an inverted pendulum. This application is developed as part of the second homework assignment in the course SES 598: Autonomous Exploration Systems at Arizona State University. The LQR function is provided by the python-controls toolbox.

Code was tested on Ubuntu 20.04 running ROS Noetic.

## Demonstration
![py_lqr_invpend_demo(1)](https://user-images.githubusercontent.com/82643627/155905754-6a210cf4-668e-4f56-aaa3-dbf174dc7be0.gif)

## To Install Python Control Toolbox
```bash
pip3 install control
```

## To Install ROS Package

This package relies on a Gazebo simulation of an inverted pendulum. Please visit the follow repository and follow their instructions for setting up the simulation components: https://github.com/linZHank/invpend_experiment.

```bash
cd ~/catkin_ws/src
https://github.com/Mannat-Rana/LQR-Control-For-Inverted_Pendulum.git
catkin build lqr_invpend_control
cd ~/catkwin_ws
source devel/setup.bash
```

## To Run
```bash
roslaunch lqr_invpend_control lqr_invpend_control.launch
```
Once the simulation opens, select the pole link, and apply forces and torques to it as desired for testing. Robustness of the controller can be tweaked by modifying the Q and R matrices in the [source file](https://github.com/Mannat-Rana/LQR-Control-For-Inverted_Pendulum/blob/main/lqr_invpend_control/src/lqr_invpend_control.py).
