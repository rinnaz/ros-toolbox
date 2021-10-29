#include "rnrt_control_tools/state_space_model.h"

StateSpaceModel::StateSpaceModel(const TransferFcn &tfcn)
    : m_denominator_size{tfcn.getDenominator().size()},
      m_matrix_size{m_denominator_size - 1},
      m_current_state(Eigen::VectorXd::Zero(m_matrix_size)),
      m_A_matrix(m_matrix_size, m_matrix_size),
      m_B_vector(m_matrix_size),
      m_C_vector(m_matrix_size),
      m_denominator{[&]
                    {
                        Eigen::VectorXd ret{Eigen::VectorXd::Zero(m_denominator_size)};
                        auto input{tfcn.getDenominator()};
                        auto divisor{tfcn.getDenominator().at(0)};
                        for (auto i : input)
                        {
                            ret << i / divisor;
                        }
                        return ret;
                    }()},
      m_numerator{[&]
                  {
                      Eigen::VectorXd ret{Eigen::VectorXd::Zero(m_denominator_size)};
                      auto input{tfcn.getNumerator()};
                      auto divisor{tfcn.getDenominator().at(0)};
                      for (auto i{m_denominator_size - 1};
                           i >= 0;
                           i--)
                      {
                          ret(i) = input.at(i) / divisor;
                      }
                      return ret;
                  }()}
{
    m_A_matrix = setAMatrix();
    m_B_vector = setBVector();
    m_C_vector = setCRowVector();
    m_D = m_numerator(0);

    m_integrators.push_back(std::bind(&StateSpaceModel::eulerCompute,
                                      this,
                                      std::placeholders::_1,
                                      std::placeholders::_2,
                                      std::placeholders::_3));

    m_integrators.push_back(std::bind(&StateSpaceModel::rungekuttaCompute,
                                      this,
                                      std::placeholders::_1,
                                      std::placeholders::_2,
                                      std::placeholders::_3));
}

StateSpaceModel::~StateSpaceModel() {}

Eigen::MatrixXd StateSpaceModel::setAMatrix()
{
    Eigen::MatrixXd result{Eigen::MatrixXd::Zero(m_matrix_size, m_matrix_size)};

    for (auto i{0}; i < m_matrix_size; i++)
    {
        result(m_matrix_size - 1, i) = -m_denominator(m_matrix_size - i);
    }

    if (m_matrix_size >= 2)
    {
        for (auto i{0}; i < m_matrix_size - 1; i++)
        {
            result(i, i + 1) = 1.0;
        }
    }

    return result;
}

Eigen::VectorXd StateSpaceModel::setBVector()
{
    Eigen::VectorXd result{Eigen::VectorXd::Zero(m_matrix_size)};
    result(m_matrix_size - 1) = 1.0;

    return result;
}

Eigen::RowVectorXd StateSpaceModel::setCRowVector()
{
    Eigen::RowVectorXd result;

    for (auto i{0}; i < m_matrix_size; i++)
    {
        result(i) = m_numerator(m_matrix_size - i) - m_denominator(m_matrix_size - i) * m_numerator(0);
    }

    return result;
}

Eigen::VectorXd StateSpaceModel::getDerivatives(const Eigen::VectorXd &state,
                                                const double &input) const
{
    return m_A_matrix * state + m_B_vector * input;
}

double StateSpaceModel::getResponse(const Eigen::VectorXd &last_state,
                                    const double &input,
                                    const uint64_t &dt,
                                    SolverType solver) const
{
    auto i{static_cast<int>(solver)};

    return m_C_vector * m_integrators.at(i)(last_state, input, dt) + m_D * input;
}

Eigen::VectorXd StateSpaceModel::rungekuttaCompute(const Eigen::VectorXd &last_state,
                                                   const double &input,
                                                   const uint64_t &dt) const
{
    auto k1 = dt / 1e9 * getDerivatives(last_state, input);
    auto k2 = dt / 1e9 * getDerivatives(last_state + k1 / 2.0, input);
    auto k3 = dt / 1e9 * getDerivatives(last_state + k2 / 2.0, input);
    auto k4 = dt / 1e9 * getDerivatives(last_state + k3, input);

    return last_state + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6;
}

Eigen::VectorXd StateSpaceModel::eulerCompute(const Eigen::VectorXd &last_state,
                                              const double &input,
                                              const uint64_t &dt) const
{
    return last_state + dt / 1e9 * getDerivatives(last_state, input);
}