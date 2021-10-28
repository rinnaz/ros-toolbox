#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <limits>
#include <numeric>
#include "eigen3/Eigen/Core"

#include "rnrt_control_tools/transfer_fcn.h"
// #include "rnrt_control_tools/ode_solver.h"

enum class SolverType
{
    EULER,
    RUNGEKUTTA
};

// State space model representation of transfer function
class StateSpaceModel
{
public:
    StateSpaceModel(const TransferFcn& tfcn);
    ~StateSpaceModel();
    Eigen::MatrixXd setAMatrix();
    Eigen::VectorXd setBVector();
    Eigen::RowVectorXd setCRowVector();  
    Eigen::VectorXd getDerivatives(Eigen::VectorXd state, const double& input) const;
    double getResponse(const Eigen::VectorXd& last_state, 
                       const double& input,
                       const uint64_t& dt,
                       SolverType = SolverType::EULER) const;

    Eigen::VectorXd rungekuttaCompute(const Eigen::VectorXd& last_state, 
                                      const double& input,
                                      const uint64_t& dt) const;

private:
    Eigen::VectorXd m_numerator, m_denominator;
    const uint64_t m_matrix_size;
    Eigen::VectorXd m_current_state;
    Eigen::MatrixXd m_A_matrix;
    Eigen::VectorXd m_B_vector;
    Eigen::RowVectorXd m_C_vector;
    double m_D;
}; 
