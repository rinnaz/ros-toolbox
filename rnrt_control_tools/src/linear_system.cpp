#include "rnrt_control_tools/linear_system.h"

namespace control_tools
{
LinearSystem::LinearSystem()
{
}

LinearSystem::LinearSystem(const std::vector<double> &numerator, const std::vector<double> &denominator,
                           const SolverType solver)
{
  init(numerator, denominator, solver);
}

void LinearSystem::init(const std::vector<double> &numerator, const std::vector<double> &denominator,
                        const SolverType solver)
{
  m_solver = solver;
  m_tfcn = std::make_shared<TransferFcn>(numerator, denominator);

  m_model = std::make_shared<StateSpaceModel>();

  m_model->init(*m_tfcn);
}

bool LinearSystem::init(const ros::NodeHandle &n, const SolverType solver)
{
  ros::NodeHandle nh(n);
  std::vector<double> num, den;
  
  // Load system parameters from parameter server
  if (!nh.getParam("numerator", num))
  {
    ROS_ERROR("No numerator specified for transfer function.  Namespace: %s", nh.getNamespace().c_str());
    return false;
  }

  if (!nh.getParam("denominator", den))
  {
    ROS_ERROR("No denominator specified for transfer function.  Namespace: %s", nh.getNamespace().c_str());
    return false;
  }

  init(num, den, solver);

  return true;
}

double LinearSystem::computeResponse(const double &input, const uint64_t &time_step)
{
  return m_model->getResponse(input, time_step, m_solver);
}

void LinearSystem::reset()
{
  m_model->resetState();
}

}  // namespace control_tools
