#include "rnrt_control_tools/ode_solver.h"

std::ostream &operator<<(std::ostream &out, const OdeSolver &a)
{
    out << a.m_equation << std::endl
        << a.m_order;

    return out;
}

OdeSolver::OdeSolver(const std::vector<double> inputEquation)
    : m_equation {normalize(inputEquation)},
      m_order {inputEquation.size() - 2},
      m_current_state(m_order, 0.0)
{

}

void OdeSolver::initialize(const std::vector<double> inputEquation)
{
    m_equation = normalize(inputEquation);

    m_order = inputEquation.size() - 2;

    for (auto i {0}; i < m_order; i++)
    {
        m_current_state.push_back(0.0);
    }
}

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

std::vector<double> OdeSolver::normalize(const std::vector<double> inputEquation) const
{
    std::vector<double> result;

    for (auto i : inputEquation)
    {
        result.push_back(i / inputEquation.at(0));
    }

    return result;
}

std::vector<double> OdeSolver::computeDerivatives(const std::vector<double> state) const
{
    //    result = state[1:]
    auto result = std::vector<double>(state.begin() + 1, state.end());

    //    temp = eq[1:-1] * state
    auto temp = std::vector<double>(m_equation.rbegin() + 1, m_equation.rend() - 1) * state;

    //    result.append(-eq[last] - sum(temp))
    result.push_back(-m_equation.back() - accumulate(temp.begin(), temp.end(), 0.0));

    return result;
}

std::vector<double> OdeSolver::eulerCompute()
{
    return m_current_state + m_step_size * computeDerivatives(m_current_state);
}

std::vector<double> OdeSolver::rungekuttaCompute()
{
    m_k1 = m_step_size * computeDerivatives(m_current_state);
    m_k2 = m_step_size * computeDerivatives(m_current_state + m_k1 / 2.0);
    m_k3 = m_step_size * computeDerivatives(m_current_state + m_k2 / 2.0);
    m_k4 = m_step_size * computeDerivatives(m_current_state + m_k3);

    return m_current_state + (m_k1 + 2.0 * m_k2 + 2.0 * m_k3 + m_k4) / 6;
}

std::ostream &operator<<(std::ostream &out, const std::vector<double> &right)
{
    std::cout << std::setprecision(5);

    for (auto a : right)
    {
        out << a << " ";
    }

    std::cout << std::setprecision(2);

    return out;
}

const std::vector<double> operator*(const std::vector<double> &left, const std::vector<double> &right)
{
    std::vector<double> result;

    for (auto i{0}; i < left.size(); i++)
    {
        result.push_back(left.at(i) * right.at(i));
    }

    return result;
}

const std::vector<double> operator*(const double &left, const std::vector<double> &right)
{
    std::vector<double> result;
    for (auto i : right)
    {
        result.push_back(i * left);
    }

    return result;
}

const std::vector<double> operator+(const std::vector<double> &left, const std::vector<double> &right)
{
    std::vector<double> result;

    for (auto i{0}; i < left.size(); i++)
    {
        result.push_back(left.at(i) + right.at(i));
    }

    return result;
}

const std::vector<double> operator/(const std::vector<double> &left, const double &right)
{
    return (1 / right) * left;
}

const std::vector<double> operator*(const std::vector<double> &left, const double &right)
{
    return right * left;
}

const std::vector<double> operator-(const std::vector<double> &left, const std::vector<double> &right)
{
    return left + (-1) * right;
}

const std::vector<double> operator-(int, const std::vector<double> &right)
{
    return (-1) * right;
}
