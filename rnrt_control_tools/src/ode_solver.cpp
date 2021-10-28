#include "rnrt_control_tools/ode_solver.h"

void OdeSolver::solve(double computationTime, double stepValue, SolverType solver, std::ostream &out)
{
    m_step_size = stepValue;
    out << std::fixed;

    switch (solver)
    {
    case SolverType::EULER:
        compute = &OdeSolver::eulerCompute;
        break;

    case SolverType::RUNGEKUTTA:
        compute = &OdeSolver::rungekuttaCompute;
        break;
    }

    for (double i = m_step_size; i <= computationTime; i += m_step_size)
    {
        m_current_state = (this->*compute)();
        out << m_current_state << i << std::endl;
    }
}

std::vector<double> OdeSolver::eulerCompute(Eigen::VectorXd& last_state, 
                                            uint64_t& dt)
{
    return m_current_state + m_step_size * computeDerivatives(m_current_state);
}

std::vector<double> OdeSolver::rungekuttaCompute(Eigen::VectorXd& last_state, 
                                                 uint64_t& dt)
{
    auto k1 = m_step_size * computeDerivatives(m_current_state);
    auto k2 = m_step_size * computeDerivatives(m_current_state + k1 / 2.0);
    auto k3 = m_step_size * computeDerivatives(m_current_state + k2 / 2.0);
    auto k4 = m_step_size * computeDerivatives(m_current_state + k3);

    return m_current_state + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6;
}
