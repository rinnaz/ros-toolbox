#pragma once

#include <ros/ros.h>

#include <memory>
#include <vector>

#include "eigen3/Eigen/Core"
#include "rnrt_control_tools/linear_system.h"
#include "rnrt_control_tools/state_space_model.h"

namespace control_toolbox
{
class MassSpringDamper : public control_toolbox::LinearSystem
{
public:
  MassSpringDamper();
  MassSpringDamper(const double &m, const double &k, const double &zeta, const SolverType = SolverType::EULER);
  ~MassSpringDamper(){};

  bool init(const ros::NodeHandle &n, const SolverType = SolverType::EULER);

  void init(const double &m, const double &k, const double &zeta, const SolverType = SolverType::EULER);

protected:
  double mass_;
  double damping_ratio_;
  double stiffnes_;
};

}  // namespace control_toolbox
