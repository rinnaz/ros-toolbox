#pragma once

#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <vector>

#include "eigen3/Eigen/Core"

namespace control_tools
{
class TransferFcn
{
public:
  TransferFcn();
  TransferFcn(const std::vector<double>& numerator, const std::vector<double>& denominator);
  TransferFcn(const TransferFcn& tf);

  ~TransferFcn(){};

  // For numerator defined as b_n*s^n + ... + b_1*s + b_0
  // vector values should be {b_n, ..., b_1, b_0}
  void setNumerator(const std::vector<double>& numerator);

  // For denominator defined as a_n*s^n + ... + a_1*s + a_0
  // vector values should be {a_n, ..., a_1, a_0}
  void setDenominator(const std::vector<double>& denominator);

  bool isProper() const;
  bool isValid() const;

  std::vector<double> getNumerator() const;
  std::vector<double> getDenominator() const;

private:
  std::vector<double> m_numerator;
  std::vector<double> m_denominator;
  std::vector<double> removeLeadingZeros(const std::vector<double>&) const;
};

}  // namespace control_tools
