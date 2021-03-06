#pragma once

#include <ros/ros.h>

namespace control_toolbox
{
class ServoInterface
{
public:
  virtual ~ServoInterface() = default;

  virtual bool init(const ros::NodeHandle &nh, std::string &joint_name) = 0;

  virtual double getEffortResponse(const double &command, const double &position, const double &velocity,
                                   const double &effort, ros::Duration period) = 0;

  virtual void reset() = 0;

protected:
};

}  // namespace control_toolbox
