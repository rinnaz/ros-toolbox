#pragma once

#include <ros/ros.h>

#include <memory>
#include <vector>

#include "eigen3/Eigen/Core"
#include "rnrt_control_tools/linear_system.h"
#include "rnrt_control_tools/state_space_model.h"

namespace control_toolbox
{
class PmMotor : public control_toolbox::LinearSystem
{
public:
  PmMotor();
  PmMotor(const double &ind, const double &res, const double &km, const uint64_t &pole_pairs = 1,
          const SolverType = SolverType::EULER);
  ~PmMotor(){};

  bool init(const ros::NodeHandle &n, const SolverType = SolverType::EULER);

  void init(const double &ind, const double &res, const double &km, const int &pole_pairs = 1,
            const SolverType = SolverType::EULER);

  double getCurrentResponse(const double &input_voltage, const double &current_velocity, const uint64_t &dt);

  double getTorqueResponse(const double &input_voltage, const double &current_velocity, const uint64_t &dt);

  double getKm()
  {
    return km_;
  }
  double getKe()
  {
    return ke_;
  }

protected:
  double l_;   // inductance
  double r_;   // resistance
  double te_;  // electrical time constant
  double km_;  // torque constant
  double ke_;  // electromechanical constant
  int pole_pairs_;
};

}  // namespace control_toolbox
