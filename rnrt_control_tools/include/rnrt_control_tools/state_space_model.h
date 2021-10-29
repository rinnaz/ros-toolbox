#pragma once

#include <iostream>
#include <vector>
#include <memory>
#include "eigen3/Eigen/Core"
#include <functional>

#include "rnrt_control_tools/transfer_fcn.h"
// #include "rnrt_control_tools/ode_solver.h"

enum class SolverType
{
    EULER = 0,
    RUNGEKUTTA = 1
};

// State space model representation of transfer function
class StateSpaceModel
{
public:
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
    uint64_t m_denominator_size;
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
