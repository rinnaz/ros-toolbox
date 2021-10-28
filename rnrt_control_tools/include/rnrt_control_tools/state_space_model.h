#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <limits>
#include <numeric>
#include "eigen3/Eigen/Core"

#include "rnrt_control_tools/transfer_fcn.h"
#include "rnrt_control_tools/ode_solver.h"

// State space model representation of transfer function
class StateSpaceModel
{
public:
    StateSpaceModel(const TransferFcn& tfcn);
    ~StateSpaceModel();
    Eigen::MatrixXd setAMatrix(const std::vector<double>& den);
    Eigen::VectorXd setBVector(const std::vector<double>& input);
    Eigen::RowVectorXd setCRowVector(const std::vector<double>& num);  

private:
    const uint64_t m_matrix_size;
    Eigen::VectorXd m_current_state;
    Eigen::MatrixXd m_A_matrix;
    Eigen::VectorXd m_B_vector;
    Eigen::RowVectorXd m_C_vector;
    std::unique_ptr<OdeSolver> m_solver;
}; 
