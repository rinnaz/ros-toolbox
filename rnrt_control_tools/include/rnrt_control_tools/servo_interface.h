#pragma once

#include <ros/ros.h>

#include <algorithm>
#include <memory>
#include <vector>

namespace control_tools
{
class ServoInterface
{
public:
  ServoInterface(){};
  ~ServoInterface(){};

  virtual bool init(const ros::NodeHandle &nh, std::string &joint_name) = 0;

  virtual double getEffortResponse(const double &command, const double &position, const double &velocity,
                                   const double &effort, ros::Duration period) = 0;

  virtual void reset() = 0;

protected:
};

}  // namespace control_tools
