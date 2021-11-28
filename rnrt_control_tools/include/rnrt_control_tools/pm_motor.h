#pragma once

#include <ros/ros.h>

#include <memory>
#include <vector>

#include "eigen3/Eigen/Core"
#include "rnrt_control_tools/linear_system.h"
#include "rnrt_control_tools/state_space_model.h"

namespace control_tools
{
class PmMotor : public control_tools::LinearSystem
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
    return m_km;
  }
  double getKe()
  {
    return m_ke;
  }

protected:
  double m_l;   // inductance
  double m_r;   // resistance
  double m_te;  // electrical time constant
  double m_km;  // torque constant
  double m_ke;  // electromechanical constant
  int m_pole_pairs;
};

}  // namespace control_tools
