#include "rnrt_control_tools/ode_solver.h"

ostream &operator<<(ostream &out, const OdeSolver &a)
{
    out << a.m_equation << endl
        << a.m_order;

    return out;
}

OdeSolver::OdeSolver(const vector<double> list)
{
    initialize(list);
}

void OdeSolver::initialize(const vector<double> inputEquation)
{
    m_equation.reserve(inputEquation.size());
    m_equation = normalize(inputEquation);

    m_order = inputEquation.size() - 2;
    m_currentCondition.reserve(m_order);
    for (uint32_t i = 0; i < m_order; i++)
    {
        m_currentCondition.push_back(0.0);
    }
}

void OdeSolver::solve(double computationTime, double stepValue, SolverType solver, ostream &out)
{
    m_stepSize = stepValue;
    cout << std::fixed;

    switch (solver)
    {
    case SolverType::EULER:
        compute = &OdeSolver::eulerCompute;
        break;

    case SolverType::RUNGEKUTTA:
        compute = &OdeSolver::rungekuttaCompute;
        break;
    }

    for (double i = m_stepSize; i <= computationTime; i += m_stepSize)
    {
        (this->*compute)();
        cout << m_currentCondition << i << endl;
    }
}

vector<double> OdeSolver::normalize(const vector<double> inputEquation)
{
    vector<double> result;

    for (auto i : inputEquation)
    {
        result.push_back(i / inputEquation.at(0));
    }

    return result;
}

vector<double> OdeSolver::computeDerivatives(const vector<double> condition)
{
    vector<double> result;
    result.reserve(condition.size());

    //    result = condition[1:]
    result = vector<double>(condition.begin() + 1, condition.end());

    //    temp = eq[1:-1] * condition
    auto temp = vector<double>(m_equation.rbegin() + 1, m_equation.rend() - 1) * condition;

    //    result.append(-eq[last] - sum(temp))
    result.push_back(-m_equation.back() - accumulate(temp.begin(), temp.end(), 0.0));

    return result;
}

void OdeSolver::eulerCompute()
{
    m_currentCondition = m_currentCondition +
                             m_stepSize * computeDerivatives(m_currentCondition);
}

void OdeSolver::rungekuttaCompute()
{
    vector<double> k1, k2, k3, k4;
    k1.reserve(m_currentCondition.size());
    k2.reserve(m_currentCondition.size());
    k3.reserve(m_currentCondition.size());
    k4.reserve(m_currentCondition.size());

    k1 = m_stepSize * computeDerivatives(m_currentCondition);
    k2 = m_stepSize * computeDerivatives(m_currentCondition + k1 / 2.0);
    k3 = m_stepSize * computeDerivatives(m_currentCondition + k2 / 2.0);
    k4 = m_stepSize * computeDerivatives(m_currentCondition + k3);

    m_currentCondition = m_currentCondition +
                             (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6;
}

ostream &operator<<(ostream &out, const vector<double> &right)
{
    cout << std::setprecision(5);

    for (auto a : right)
    {
        out << a << " ";
    }

    cout << std::setprecision(2);

    return out;
}

const vector<double> operator*(const vector<double> &left, const vector<double> &right)
{
    vector<double> result;

    for (uint32_t i = 0; i < left.size(); i++)
    {
        result.push_back(left.at(i) * right.at(i));
    }

    return result;
}

const vector<double> operator*(const double &left, const vector<double> &right)
{
    vector<double> result;
    for (auto i : right)
    {
        result.push_back(i * left);
    }

    return result;
}

const vector<double> operator+(const vector<double> &left, const vector<double> &right)
{
    vector<double> result;

    for (uint32_t i = 0; i < left.size(); i++)
    {
        result.push_back(left.at(i) + right.at(i));
    }

    return result;
}

const vector<double> operator/(const vector<double> &left, const double &right)
{
    return (1 / right) * left;
}

const vector<double> operator*(const vector<double> &left, const double &right)
{
    return right * left;
}

const vector<double> operator-(const vector<double> &left, const vector<double> &right)
{
    return left + (-1) * right;
}

const vector<double> operator-(int, const vector<double> &right)
{
    return (-1) * right;
}
