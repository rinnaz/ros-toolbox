# Joint Trajectory Controller with effort feed forward input

Controller for executing joint-space trajectories on a group of joints.
Basically it is ROS deafault Joint Trajectory Controller but it also subscribes to effort FF topic and adds effort_FF to command
