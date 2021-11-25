#pragma once

#include <control_toolbox/pid.h>
#include <ros/ros.h>

#include <algorithm>
#include <memory>
#include <vector>

#include "rnrt_control_tools/pm_motor.h"
#include "rnrt_control_tools/servo_interface.h"

namespace control_tools
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
  double m_u_max;
  double m_current_last;
  double m_gear_ratio;
  double m_efficiency;

  using MotorPtr = std::shared_ptr<PmMotor>;
  using PidPtr = std::shared_ptr<control_toolbox::Pid>;
  using FilterPtr = std::shared_ptr<StateSpaceModel>;

  MotorPtr m_motor;
  PidPtr m_pid_current;

  // Helps with simulation instability
  FilterPtr m_input_velocity_filter;
};

}  // namespace control_tools
