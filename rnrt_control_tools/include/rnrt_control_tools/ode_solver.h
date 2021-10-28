#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <limits>
#include <numeric>
#include "eigen3/Eigen/Core"

enum class SolverType
{
    EULER,
    RUNGEKUTTA
};

class OdeSolver
{
public:
    OdeSolver(){};
    ~OdeSolver(){};
    
    void solve(Eigen::VectorXd& last_state, SolverType = SolverType::EULER, std::ostream &out = std::cout);
    Eigen::VectorXd getResponse(Eigen::VectorXd& last_state, 
                                uint64_t& dt,
                                SolverType = SolverType::EULER);

private:
    std::vector<double> m_equation;
    std::vector<double> m_current_state;
    uint64_t m_order;
    double m_step_size;
    std::vector<double> m_k1, m_k2, m_k3, m_k4;

    std::vector<double> eulerCompute(Eigen::VectorXd& last_state, 
                                     uint64_t& dt);
    Eigen::VectorXd rungekuttaCompute(Eigen::VectorXd& last_state, 
                                      uint64_t& dt);
    Eigen::VectorXd (OdeSolver::*compute)(Eigen::VectorXd& last_state, 
                                          uint64_t& dt);
};

