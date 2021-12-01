#include <iostream>
#include <vector>

#include "rnrt_control_tools/linear_system.h"
#include "ros/ros.h"
#include <cmath>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_tools_node");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  control_tools::LinearSystem linsys;
  std::vector<double> num({ 1.0, 1.0 });
  std::vector<double> den({ 1.0, 1.0, 1.0 });

  linsys.init(num, den, control_tools::SolverType::EULER);

  auto start = ros::Time::now();

  double dump;

  for (auto i{ 0 }; i < 1e9; i++)
  {
    dump = linsys.computeResponse(1.0+5*sin(i/1000.0), uint64_t(1e6));
  }

  ROS_INFO_STREAM("Euler time = " << (ros::Time::now() - start).toSec());

  linsys.reset();
  linsys.init(num, den, control_tools::SolverType::RUNGEKUTTA);

  start = ros::Time::now();
  double dump2;
  for (auto i{ 0 }; i < 1e9; i++)
  {
    dump2 = linsys.computeResponse(1.0+5*sin(i/1000.0), uint64_t(1e6));
  }

  ROS_INFO_STREAM("RUNGEKUTTA time = " << (ros::Time::now() - start).toSec());
  ROS_INFO_STREAM("Delta = " << dump2-dump);
  ros::waitForShutdown();
  return 0;
}
