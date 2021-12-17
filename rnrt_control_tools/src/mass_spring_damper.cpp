#include "rnrt_control_tools/mass_spring_damper.h"

namespace control_toolbox
{
MassSpringDamper::MassSpringDamper()
{
}

MassSpringDamper::MassSpringDamper(const double &m, const double &k, const double &zeta, const SolverType solver)
{
  init(m, k, zeta, solver);
}

bool MassSpringDamper::init(const ros::NodeHandle &n, const SolverType solver)
{
  ros::NodeHandle nh(n);
  double m, zeta, k;

  // Load motor parameters from parameter server
  if (!nh.getParam("m", m))
  {
    ROS_ERROR("No mass specified for MassSpringDamper system.  Namespace: %s", nh.getNamespace().c_str());
    return false;
  }

  if (!nh.getParam("zeta", zeta))
  {
    ROS_ERROR("No damping ratio specified for MassSpringDamper system.  Namespace: %s", nh.getNamespace().c_str());
    return false;
  }

  if (!nh.getParam("k", k))
  {
    ROS_ERROR("No stiffness specified for MassSpringDamper system.  Namespace: %s", nh.getNamespace().c_str());
    return false;
  }

  init(m, k, zeta, solver);

  return true;
}

void MassSpringDamper::init(const double &m, const double &k, const double &zeta, const SolverType solver)
{
  if (m <= 0.0 || k <= 0.0 || zeta < 0.0)
  {
    throw std::range_error("Negative input is not allowed");
  }

  mass_ = m;
  damping_ratio_ = zeta;
  stiffnes_ = k;

  double damping{ 2 * sqrt(mass_ * stiffnes_) * damping_ratio_ };

  std::vector<double> num{ 1.0 };
  std::vector<double> den{ mass_, damping, stiffnes_ };

  LinearSystem::init(num, den, solver);
}

}  // namespace control_toolbox
