#pragma once

#include <iostream>
#include <iomanip>
#include <vector>
#include <limits>
#include <numeric>
#include <exception>
#include "eigen3/Eigen/Core"

class TransferFcn
{
public:
    TransferFcn();
    TransferFcn(const std::vector<double>& num, const std::vector<double>& den);

    ~TransferFcn(){};
    
    // For numerator defined as b_n*s^n + ... + b_1*s + b_0
    // vector values should be {b_n, ..., b_1, b_0}
    void setNumerator(const std::vector<double>& num);

    // For denominator defined as a_n*s^n + ... + a_1*s + a_0
    // vector values should be {a_n, ..., a_1, a_0}
    void setDenominator(const std::vector<double>& den);
    
    std::vector<double> getNumerator() const;
    std::vector<double> getDenominator() const;

private:
    std::vector<double> m_numerator;
    std::vector<double> m_denominator;

    void checkSizes();
};
