/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, Rinat Nazarov
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
  Author: Rinat Nazarov
  Desc: Implements a state space model for solving ODEs
*/

#include "rnrt_control_tools/state_space_model.h"

namespace control_toolbox
{
StateSpaceModel::StateSpaceModel()
{
}

StateSpaceModel::StateSpaceModel(const TransferFunctionInfo &tfcn)
{
  init(tfcn);
}

StateSpaceModel::StateSpaceModel(const std::vector<double> &num, const std::vector<double> &den)
{
  init(num, den);
}

void StateSpaceModel::init(const TransferFunctionInfo &tfcn)
{
  if (!tfcn.isProper())
  {
    throw std::invalid_argument("Not proper transfer function input");
  }

  matrix_size_ = tfcn.getDenominator().size() - 1;

  // converting from std::vector to Eigen::VectorXd
  // and dividing dy denominator's highest power value
  numerator_ = makeNumerator(tfcn);
  denominator_ = makeDenominator(tfcn);

  // Setting state model matrices and vectors
  A_matrix_ = calcMatrixA();
  B_matrix_ = calcMatrixB();
  C_matrix_ = calcMatrixC();
  D_matrix_ = calcMatrixD();

  // Setting
  current_state_ = Eigen::VectorXd::Zero(matrix_size_);

  // Filling the container of integration methods
  integrators_.push_back(std::bind(&StateSpaceModel::integrateEuler, this, std::placeholders::_1,
                                    std::placeholders::_2, std::placeholders::_3));

  integrators_.push_back(std::bind(&StateSpaceModel::integrateRK4, this, std::placeholders::_1, std::placeholders::_2,
                                    std::placeholders::_3));
}

void StateSpaceModel::init(const std::vector<double> &num, const std::vector<double> &den)
{
  TransferFunctionInfo tfcn{ num, den };
  init(tfcn);
}

StateSpaceModel::~StateSpaceModel()
{
}

Eigen::VectorXd StateSpaceModel::makeNumerator(const TransferFunctionInfo &tfcn) const
{
  auto input{ tfcn.getNumerator() };
  auto divisor{ tfcn.getDenominator().at(0) };
  auto offset{ tfcn.getDenominator().size() - input.size() };

  Eigen::VectorXd ret{ Eigen::VectorXd::Zero(tfcn.getDenominator().size()) };

  for (auto i{ offset }; i < tfcn.getDenominator().size(); i++)
  {
    ret(i) = input.at(i - offset) / divisor;
  }
  return ret;
}

Eigen::VectorXd StateSpaceModel::makeDenominator(const TransferFunctionInfo &tfcn) const
{
  auto input{ tfcn.getDenominator() };
  auto divisor{ tfcn.getDenominator().at(0) };

  Eigen::VectorXd ret{ Eigen::VectorXd::Zero(input.size()) };

  for (auto i{ 0 }; i < input.size(); i++)
  {
    ret(i) = input.at(i) / divisor;
  }
  return ret;
}

Eigen::MatrixXd StateSpaceModel::calcMatrixA() const
{
  // "A" matrix looks like
  //
  //  |   0   1   0   0   |
  //  |   0   0   1   0   |
  //  |   0   0   0   1   |
  //  | -a0 -a1 -a2 -a3   |

  Eigen::MatrixXd result{ Eigen::MatrixXd::Zero(matrix_size_, matrix_size_) };

  for (auto i{ 0 }; i < matrix_size_; i++)
  {
    result(matrix_size_ - 1, i) = -denominator_(matrix_size_ - i);
  }

  if (matrix_size_ >= 2)
  {
    for (auto i{ 0 }; i < matrix_size_ - 1; i++)
    {
      result(i, i + 1) = 1.0;
    }
  }
  return result;
}

Eigen::MatrixXd StateSpaceModel::calcMatrixB() const
{
  // "B" is [0, 0, ... 0, 1].T
  Eigen::VectorXd result{ Eigen::VectorXd::Zero(matrix_size_) };
  result(matrix_size_ - 1) = 1.0;
  return result;
}

Eigen::MatrixXd StateSpaceModel::calcMatrixC() const
{
  // "C" is [b0-a0*bn, b1-a1*bn ... b(n-1)-a(n-1)*bn]
  Eigen::RowVectorXd result{ Eigen::RowVectorXd::Zero(matrix_size_) };

  for (auto i{ 0 }; i < matrix_size_; i++)
  {
    result(i) = numerator_(matrix_size_ - i) - denominator_(matrix_size_ - i) * numerator_(0);
  }
  return result;
}

Eigen::MatrixXd StateSpaceModel::calcMatrixD() const
{
  // "C" is [b0-a0*bn, b1-a1*bn ... b(n-1)-a(n-1)*bn]
  Eigen::MatrixXd result{ Eigen::MatrixXd::Zero(1, 1) };
  result(0) = numerator_(0);

  return result;
}

Eigen::VectorXd StateSpaceModel::computeDerivatives(const Eigen::VectorXd &state, const double &input) const
{
  return A_matrix_ * state + B_matrix_ * input;
}

double StateSpaceModel::computeResponse(const Eigen::VectorXd &last_state, const double &input, const uint64_t &dt,
                                        SolverType solver)
{
  auto i{ static_cast<uint>(solver) };
  current_state_ = integrators_[i](last_state, input, dt);
  return (C_matrix_ * current_state_ + D_matrix_ * input)(0);
}

double StateSpaceModel::computeResponse(const double &input, const uint64_t &dt, SolverType solver)
{
  return computeResponse(current_state_, input, dt, solver);
}

Eigen::VectorXd StateSpaceModel::integrateRK4(const Eigen::VectorXd &last_state, const double &input,
                                              const uint64_t &dt) const
{
  k1_ = dt / 1e9 * computeDerivatives(last_state, input);
  k2_ = dt / 1e9 * computeDerivatives(last_state + k1_ / 2.0, input);
  k3_ = dt / 1e9 * computeDerivatives(last_state + k2_ / 2.0, input);
  k4_ = dt / 1e9 * computeDerivatives(last_state + k3_, input);

  return last_state + (k1_ + 2.0 * k2_ + 2.0 * k3_ + k4_) / 6;
}

Eigen::VectorXd StateSpaceModel::integrateEuler(const Eigen::VectorXd &last_state, const double &input,
                                                const uint64_t &dt) const
{
  return last_state + dt / 1e9 * computeDerivatives(last_state, input);
}

void StateSpaceModel::resetState()
{
  current_state_ = Eigen::VectorXd::Zero(matrix_size_);
}

Eigen::MatrixXd StateSpaceModel::getMatrixA()
{
  return A_matrix_;
}

Eigen::MatrixXd StateSpaceModel::getMatrixB()
{
  return B_matrix_;
}

}  // namespace control_toolbox
