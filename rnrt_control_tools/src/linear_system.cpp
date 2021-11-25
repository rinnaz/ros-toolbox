#include "rnrt_control_tools/linear_system.h"

namespace control_tools
{

LinearSystem::LinearSystem(const std::vector<double> &numerator, const std::vector<double> &denominator, SolverType solver)
{
    init(numerator, denominator, solver);
}

void LinearSystem::init(const std::vector<double> &numerator, const std::vector<double> &denominator, SolverType solver)
{
  m_solver = solver;
  m_tfcn = std::make_shared<TransferFcn>(numerator, denominator);

  m_model = std::make_shared<StateSpaceModel>();

  m_model->init(*m_tfcn);
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
