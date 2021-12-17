#include "rnrt_control_tools/current_servo.h"

namespace control_toolbox
{
CurrentServo::CurrentServo() : current_last_{ 0.0 }
{
}

bool CurrentServo::init(const ros::NodeHandle &nh, std::string &joint_name)
{
  double u_max, gear_ratio, eff;

  const ros::NodeHandle nh_inv(nh, "inverter/" + joint_name);
  const ros::NodeHandle nh_gear(nh, "gear/" + joint_name);

  if (!nh_inv.getParam("umax", u_max))
  {
    ROS_ERROR("No U_max specified for motor.  Namespace: %s", nh_inv.getNamespace().c_str());
    return false;
  }

  if (!nh_gear.getParam("ratio", gear_ratio))
  {
    ROS_ERROR("No gear_ratio specified for motor.  Namespace: %s", nh_gear.getNamespace().c_str());
    return false;
  }

  if (!nh_gear.getParam("eff", eff))
  {
    ROS_ERROR("No efficiency specified for motor.  Namespace: %s", nh_gear.getNamespace().c_str());
    return false;
  }

  init(u_max, gear_ratio, nh, joint_name, eff);

  return true;
}

void CurrentServo::init(double &u_max, double &gear_ratio, const ros::NodeHandle &nh, std::string &joint_name,
                        const double &efficiency)
{
  if (u_max <= 0.0)
  {
    ROS_WARN_STREAM("U_max is out of range");
    return;
  }

  if (gear_ratio <= 0.0)
  {
    ROS_WARN_STREAM("Gear_ratio is out of range");
    return;
  }

  u_max_ = u_max;
  gear_ratio_ = gear_ratio;
  efficiency_ = efficiency;

  if (efficiency > 1.0 || efficiency <= 0.0)
  {
    ROS_WARN_STREAM("Efficiency is out of range, setting to 1.0");
    efficiency_ = 1.0;
  }

  ros::NodeHandle motor_nh(nh, "motor_parameters/" + joint_name);
  ros::NodeHandle pid_nh(nh, "current_loop_gains/" + joint_name);

  initPid(pid_nh);
  initMotor(motor_nh);
  TransferFunctionInfo tfcn{ { 1.0 }, { 0.01, 1.0 } };
  input_velocity_filter_ = std::make_shared<StateSpaceModel>(tfcn);
}

void CurrentServo::initPid(const ros::NodeHandle &n)
{
  pid_current_ = std::make_shared<control_toolbox::Pid>();
  if (!pid_current_->init(n))
  {
    ROS_WARN_STREAM("Failed to initialize PID gains from ROS parameter server.");
    return;
  }
}

void CurrentServo::initMotor(const ros::NodeHandle &n)
{
  motor_ = std::make_shared<PmMotor>();
  if (!motor_->init(n))
  {
    ROS_WARN_STREAM("Failed to initialize motor from ROS parameter server.");
    return;
  }
}

double CurrentServo::getEffortResponse(const double &effort_command, const double &position, const double &velocity,
                                       const double &effort, ros::Duration period)
{
  auto current_command{ ((effort_command / gear_ratio_) / motor_->getKm()) - current_last_ };

  auto voltage_command{ pid_current_->computeCommand(current_command, period) };

  auto rotor_velocity = velocity * gear_ratio_;

  // rotor_velocity = input_velocity_filter_->getResponse(rotor_velocity,
  //                                                       period.toNSec(),
  //                                                       SolverType::RK4);

  voltage_command = std::clamp(voltage_command, -u_max_, u_max_);

  current_last_ = motor_->getCurrentResponse(voltage_command, rotor_velocity, period.toNSec());

  // current_last_ = std::clamp(current_last_, -u_max_/2, u_max_/2);

  return current_last_ * motor_->getKm() * gear_ratio_ * efficiency_;
}

void CurrentServo::reset()
{
  current_last_ = 0.0;
  pid_current_->reset();
  motor_->reset();
}

}  // namespace control_toolbox
