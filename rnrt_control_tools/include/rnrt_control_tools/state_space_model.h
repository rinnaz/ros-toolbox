#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include "eigen3/Eigen/Core"
#include <functional>

#include "rnrt_control_tools/transfer_fcn.h"

enum class SolverType
{
    EULER = 0,
    RUNGEKUTTA = 1
};

// State space model representation of transfer function
// Transfer function is defined with numerator and denominator
//          b_n*s^n + ... + b_1*s + b_0
//       ---------------------------------
//          a_n*s^n + ... + a_1*s + a_0
//
// And state space model form is
//              x' = A * x + B * u(t)
//             y(t) = C * x + D * u(t)
// where "x" is state (vector), "u" is input (scalar), "y" is response (scalar)
// "A" is matrix, "B" - vector, "C" - row vector, D - scalar
class StateSpaceModel
{
public:
    StateSpaceModel();
    StateSpaceModel(const TransferFcn &tfcn);
    ~StateSpaceModel();

    double getResponse(const Eigen::VectorXd &last_state,
                       const double &input,
                       const uint64_t &dt,
                       const SolverType = SolverType::EULER);

    double getResponse(const double &input,
                       const uint64_t &dt,
                       const SolverType = SolverType::EULER);
    
    void resetState();

private:
    Eigen::VectorXd makeNumerator(const TransferFcn &tfcn) const;
    Eigen::VectorXd makeDenominator(const TransferFcn &tfcn) const;
    Eigen::MatrixXd calcAMatrix() const;
    Eigen::VectorXd calcBVector() const;
    Eigen::RowVectorXd calcCRowVector() const;
    Eigen::VectorXd getDerivatives(const Eigen::VectorXd &state,
                                   const double &input) const;

    Eigen::VectorXd eulerCompute(const Eigen::VectorXd &last_state,
                                 const double &input,
                                 const uint64_t &dt) const;

    Eigen::VectorXd rungekuttaCompute(const Eigen::VectorXd &last_state,
                                      const double &input,
                                      const uint64_t &dt) const;

    Eigen::VectorXd m_numerator, m_denominator;
    uint64_t m_matrix_size;
    Eigen::VectorXd m_current_state;
    Eigen::MatrixXd m_A_matrix;
    Eigen::VectorXd m_B_vector;
    Eigen::RowVectorXd m_C_vector;
    double m_D;

    std::vector<std::function<Eigen::VectorXd(const Eigen::VectorXd &,
                                              const double &,
                                              const uint64_t &)>>
        m_integrators;
};
