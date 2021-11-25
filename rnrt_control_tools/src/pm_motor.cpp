#include "rnrt_control_tools/pm_motor.h"

namespace control_tools
{
PmMotor::PmMotor() : m_l{ 1.0 }, m_r{ 1.0 }, m_te{ m_l / m_r }, m_km{ 1.0 }, m_ke{ 1.0 }, m_pole_pairs{ 1 }
{
  initStateSpaceModel();
}

PmMotor::PmMotor(const double &ind, const double &res, const double &km, const uint64_t &pole_pairs)
{
  init(ind, res, km, pole_pairs);
}

bool PmMotor::init(const ros::NodeHandle &n)
{
  ros::NodeHandle nh(n);
  double l, r, km;
  int pole_pairs;

  // Load PID gains from parameter server
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

  init(l, r, km, pole_pairs);

  return true;
}

void PmMotor::init(const double &ind, const double &res, const double &km, const int &pole_pairs)
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

  initStateSpaceModel();
}

void PmMotor::initStateSpaceModel()
{
  TransferFcn tfcn{ { 1.0 / m_r }, { m_te, 1.0 } };
  m_state_space_model_ptr = std::make_shared<StateSpaceModel>(tfcn);
}

double PmMotor::getCurrentResponse(const double &input_voltage, const double &current_velocity, const uint64_t &dt,
                                   const SolverType solver)
{
  return m_state_space_model_ptr->getResponse(input_voltage - current_velocity * m_ke, dt, solver);
}

double PmMotor::getTorqueResponse(const double &input_voltage, const double &current_velocity, const uint64_t &dt,
                                  const SolverType solver)
{
  return m_km * getCurrentResponse(input_voltage, current_velocity, dt, solver);
}

void PmMotor::reset()
{
  m_state_space_model_ptr->resetState();
}

}  // namespace control_tools
