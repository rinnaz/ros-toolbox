# RN ROS Toolbox
ROS packages for my projects

## rnrt_aruco_detector
ROS node for detecting aruco markers. Publishes markers poses and tfs, so they could be processed and viewed via rviz.

## rnrt_control_tools
At the moment it contains classes for: 
 - transfer function description
 - state space model (Euler and RK4 solvers)
 - linear dynamic system
 - Butterworth filter
 - permanent magnet brushed motor
 - servo motor with current loop
 - mass-spring-damper model

## rnrt_joint_trajectory_controller
Implementation of joint_trajectory_controller with effort feed forward input

## rnrt_robot_controllers
Robot-level controllers. Currently has:
 - gravity compensation controller

## rnrt_gazebo_ros_control
Hardware interface plugin to simulate servo motor actuator
