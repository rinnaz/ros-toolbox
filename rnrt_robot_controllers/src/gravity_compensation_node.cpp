#include "rnrt_robot_controllers/gravity_compensation_controller.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "iiwa_controller");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_WARN("iiwa_controller process started");

  GravityCompenationController controller;

  ros::waitForShutdown();
  return 0;
}
 
