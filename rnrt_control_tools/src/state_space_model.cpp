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
 *   * Neither the name of the Willow Garage nor the names of its
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

  m_matrix_size = tfcn.getDenominator().size() - 1;
  
  // converting from std::vector to Eigen::VectorXd
  // and dividing dy denominator's highest power value
  m_numerator = makeNumerator(tfcn);
  m_denominator = makeDenominator(tfcn);

  // Setting state model matrices and vectors
  m_A_matrix = calcAMatrix();
  m_B_vector = calcBVector();
  m_C_vector = calcCRowVector();
  m_D = m_numerator(0);

  // Setting
  m_current_state = Eigen::VectorXd::Zero(m_matrix_size);

  // Filling the container of integration methods
  m_integrators.push_back(std::bind(&StateSpaceModel::integrateEuler, this, std::placeholders::_1, std::placeholders::_2,
                                    std::placeholders::_3));

  m_integrators.push_back(std::bind(&StateSpaceModel::integrateRK4, this, std::placeholders::_1,
                                    std::placeholders::_2, std::placeholders::_3));
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

Eigen::MatrixXd StateSpaceModel::calcAMatrix() const
{
  // "A" matrix looks like
  //
  //  |   0   1   0   0   |
  //  |   0   0   1   0   |
  //  |   0   0   0   1   |
  //  | -a0 -a1 -a2 -a3   |

  Eigen::MatrixXd result{ Eigen::MatrixXd::Zero(m_matrix_size, m_matrix_size) };

  for (auto i{ 0 }; i < m_matrix_size; i++)
  {
    result(m_matrix_size - 1, i) = -m_denominator(m_matrix_size - i);
  }

  if (m_matrix_size >= 2)
  {
    for (auto i{ 0 }; i < m_matrix_size - 1; i++)
    {
      result(i, i + 1) = 1.0;
    }
  }
  return result;
}

Eigen::VectorXd StateSpaceModel::calcBVector() const
{
  // "B" is [0, 0, ... 0, 1].T
  Eigen::VectorXd result{ Eigen::VectorXd::Zero(m_matrix_size) };
  result(m_matrix_size - 1) = 1.0;
  return result;
}

Eigen::RowVectorXd StateSpaceModel::calcCRowVector() const
{
  // "C" is [b0-a0*bn, b1-a1*bn ... b(n-1)-a(n-1)*bn]
  Eigen::RowVectorXd result{ Eigen::RowVectorXd::Zero(m_matrix_size) };

  for (auto i{ 0 }; i < m_matrix_size; i++)
  {
    result(i) = m_numerator(m_matrix_size - i) - m_denominator(m_matrix_size - i) * m_numerator(0);
  }
  return result;
}

Eigen::VectorXd StateSpaceModel::computeDerivatives(const Eigen::VectorXd &state, const double &input) const
{
  return m_A_matrix * state + m_B_vector * input;
}

double StateSpaceModel::computeResponse(const Eigen::VectorXd &last_state, const double &input, const uint64_t &dt,
                                    SolverType solver)
{
  auto i{ static_cast<int>(solver) };
  m_current_state = m_integrators.at(i)(last_state, input, dt);
  return m_C_vector * m_current_state + m_D * input;
}

double StateSpaceModel::computeResponse(const double &input, const uint64_t &dt, SolverType solver)
{
  return computeResponse(m_current_state, input, dt, solver);
}

Eigen::VectorXd StateSpaceModel::integrateRK4(const Eigen::VectorXd &last_state, const double &input,
                                                   const uint64_t &dt) const
{
  m_k1 = dt / 1e9 * computeDerivatives(last_state, input);
  m_k2 = dt / 1e9 * computeDerivatives(last_state + m_k1 / 2.0, input);
  m_k3 = dt / 1e9 * computeDerivatives(last_state + m_k2 / 2.0, input);
  m_k4 = dt / 1e9 * computeDerivatives(last_state + m_k3, input);

  return last_state + (m_k1 + 2.0 * m_k2 + 2.0 * m_k3 + m_k4) / 6;
}

Eigen::VectorXd StateSpaceModel::integrateEuler(const Eigen::VectorXd &last_state, const double &input,
                                              const uint64_t &dt) const
{
  return last_state + dt / 1e9 * computeDerivatives(last_state, input);
}

void StateSpaceModel::resetState()
{
  m_current_state = Eigen::VectorXd::Zero(m_matrix_size);
}

}  // namespace control_toolbox
