#pragma once

#include <memory>
#include <vector>

#include "rnrt_control_tools/state_space_model.h"
#include "rnrt_control_tools/transfer_fcn.h"

namespace control_tools
{
class LinearSystem
{
public:
  LinearSystem(){};
  LinearSystem(const std::vector<double> &numerator, const std::vector<double> &denominator, SolverType solver);
  ~LinearSystem(){};

  void init(const std::vector<double> &numerator, const std::vector<double> &denominator,
            SolverType solver = SolverType::EULER);

  double computeResponse(const double &input, const uint64_t &time_step);

  void reset();

protected:
  SolverType m_solver;
  std::shared_ptr<control_tools::TransferFcn> m_tfcn;
  std::shared_ptr<control_tools::StateSpaceModel> m_model;
};

}  // namespace control_tools
