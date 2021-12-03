# RN ROS Toolbox
Several control tools: continuous domain state space model solvers, effort feed-forward trajectory controller, gravity compensation controller, Gazebo hardware interface plugin to simulate servo drives and etc.

## Dependencies
All packages are built against ROS Noetic. You should have installed: moveit, gazebo_ros packages, ros_controll and ros_controllers packages (**TBD**)

External dependencies: Eigen, OpenCV

## Package details

### rnrt_aruco_detector
ROS node for detecting aruco markers. Publishes markers poses and tfs, so they could be processed and viewed via rviz.

### rnrt_control_tools
At the moment it contains classes for: 
 - transfer function description
 - state space model (Euler and RK4 solvers)
 - linear dynamic system
 - Butterworth filter
 - permanent magnet brushed motor
 - servo motor with current loop
 - mass-spring-damper model

### rnrt_joint_trajectory_controller
Implementation of joint_trajectory_controller with effort feed forward input

### rnrt_robot_controllers
Robot-level controllers. Currently has:
 - gravity compensation controller

### rnrt_gazebo_ros_control
Hardware interface plugin to simulate servo motor actuator
