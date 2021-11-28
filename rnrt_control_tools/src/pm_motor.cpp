#include "rnrt_control_tools/pm_motor.h"

namespace control_tools
{
PmMotor::PmMotor()
{
}

PmMotor::PmMotor(const double &ind, const double &res, const double &km, const uint64_t &pole_pairs,
                 const SolverType solver)
{
  init(ind, res, km, pole_pairs, solver);
}

bool PmMotor::init(const ros::NodeHandle &n, const SolverType solver)
{
  ros::NodeHandle nh(n);
  double l, r, km;
  int pole_pairs;

  // Load motor parameters from parameter server
  if (!nh.getParam("l", l))
  {
    ROS_ERROR("No inductance specified for motor.  Namespace: %s", nh.getNamespace().c_str());
    return false;
  }

  if (!nh.getParam("r", r))
  {
    ROS_ERROR("No resistance specified for motor.  Namespace: %s", nh.getNamespace().c_str());
    return false;
  }

  if (!nh.getParam("km", km))
  {
    ROS_ERROR("No torque constant specified for motor.  Namespace: %s", nh.getNamespace().c_str());
    return false;
  }

  // Pole pairs number is optional and default to 1:
  nh.param("pole_pairs", pole_pairs, 1);

  init(l, r, km, pole_pairs, solver);

  return true;
}

void PmMotor::init(const double &ind, const double &res, const double &km, const int &pole_pairs,
                   const SolverType solver)
{
  if (ind <= 0.0 || res <= 0.0 || km <= 0.0 || pole_pairs <= 0)
  {
    throw std::range_error("Negative input is not allowed");
  }
  m_l = ind;
  m_r = res;
  m_km = km;
  m_ke = km;
  m_te = m_l / m_r;
  m_pole_pairs = pole_pairs;

  std::vector<double> num{ 1.0 / m_r };
  std::vector<double> den{ m_te, 1.0 };

  LinearSystem::init(num, den, solver);
}

double PmMotor::getCurrentResponse(const double &input_voltage, const double &current_velocity, const uint64_t &dt)
{
  return LinearSystem::computeResponse(input_voltage - current_velocity * m_ke, dt);
}

double PmMotor::getTorqueResponse(const double &input_voltage, const double &current_velocity, const uint64_t &dt)
{
  return m_km * getCurrentResponse(input_voltage, current_velocity, dt);
}

}  // namespace control_tools
