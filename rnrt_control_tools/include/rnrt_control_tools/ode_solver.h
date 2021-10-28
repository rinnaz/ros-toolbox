#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <limits>
#include <numeric>
#include "eigen3/Eigen/Core"

std::ostream &operator<<(std::ostream &out, const std::vector<double> &right);
const std::vector<double> operator*(const std::vector<double> &left, const std::vector<double> &right);
const std::vector<double> operator*(const double &left, const std::vector<double> &right);
const std::vector<double> operator*(const std::vector<double> &left, const double &right);
const std::vector<double> operator/(const std::vector<double> &right, const double &left);
const std::vector<double> operator+(const std::vector<double> &left, const std::vector<double> &right);
const std::vector<double> operator-(const std::vector<double> &left, const std::vector<double> &right);
const std::vector<double> operator-(int, const std::vector<double> &right);

enum class SolverType
{
    EULER,
    RUNGEKUTTA
};

class OdeSolver
{
public:
    //    methods
    friend std::ostream &operator<<(std::ostream &out, const OdeSolver &a);

    OdeSolver();
    OdeSolver(const std::vector<double> inputEquation);
    void initialize(const std::vector<double> inputEquation);
    void solve(double computationTime, double stepValue, SolverType = SolverType::EULER, std::ostream &out = std::cout);
    double getResponse(const std::vector<double>& u, uint64_t dt);

private:
    std::vector<double> m_equation;
    std::vector<double> m_current_state;
    uint64_t m_order;
    double m_step_size;
    std::vector<double> m_k1, m_k2, m_k3, m_k4;

    std::vector<double> normalize(const std::vector<double> inputEquation) const;
    std::vector<double> computeDerivatives(const std::vector<double> state) const;
    std::vector<double> eulerCompute();
    std::vector<double> rungekuttaCompute();
    std::vector<double> (OdeSolver::*compute)();
};

