#pragma once

#include <control_toolbox/pid.h>
#include <ros/ros.h>

#include <algorithm>
#include <memory>
#include <vector>

#include "rnrt_control_tools/pm_motor.h"
#include "rnrt_control_tools/servo_interface.h"

namespace control_toolbox
{
class CurrentServo : public ServoInterface
{
public:
  CurrentServo();
  ~CurrentServo(){};

  bool init(const ros::NodeHandle &nh, std::string &joint_name) override;

  void init(double &u_max, double &gear_ratio, const ros::NodeHandle &nh, std::string &joint_name,
            const double &efficiency = 1.0);
  void initPid(const ros::NodeHandle &n);
  void initMotor(const ros::NodeHandle &n);

  double getEffortResponse(const double &effort_command, const double &position, const double &velocity,
                           const double &effort, ros::Duration period) override;

  void reset() override;

protected:
  double u_max_;
  double current_last_;
  double gear_ratio_;
  double efficiency_;

  using MotorPtr = std::shared_ptr<PmMotor>;
  using PidPtr = std::shared_ptr<control_toolbox::Pid>;
  using FilterPtr = std::shared_ptr<StateSpaceModel>;

  MotorPtr motor_;
  PidPtr pid_current_;

  // Helps with simulation instability
  FilterPtr input_velocity_filter_;
};

}  // namespace control_toolbox
