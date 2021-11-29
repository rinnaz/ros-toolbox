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


#pragma once

#include <functional>
#include <iostream>
#include <memory>
#include <vector>

#include "eigen3/Eigen/Core"
#include "rnrt_control_tools/transfer_fcn.h"

namespace control_toolbox
{
enum class SolverType
{
  EULER = 0,
  RUNGEKUTTA = 1
};

// State space model representation of transfer function of continuous dynamic system
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
  StateSpaceModel(const std::vector<double> &numerator, const std::vector<double> &denominator);
  ~StateSpaceModel();

  void init(const TransferFcn &tfcn);
  void init(const std::vector<double> &numerator, const std::vector<double> &denominator);

  double getResponse(const Eigen::VectorXd &last_state, const double &input, const uint64_t &dt,
                     const SolverType = SolverType::EULER);

  double getResponse(const double &input, const uint64_t &dt, const SolverType = SolverType::EULER);

  void resetState();

private:
  Eigen::VectorXd makeNumerator(const TransferFcn &tfcn) const;
  Eigen::VectorXd makeDenominator(const TransferFcn &tfcn) const;
  Eigen::MatrixXd calcAMatrix() const;
  Eigen::VectorXd calcBVector() const;
  Eigen::RowVectorXd calcCRowVector() const;
  Eigen::VectorXd getDerivatives(const Eigen::VectorXd &state, const double &input) const;

  Eigen::VectorXd eulerCompute(const Eigen::VectorXd &last_state, const double &input, const uint64_t &dt) const;

  Eigen::VectorXd rungekuttaCompute(const Eigen::VectorXd &last_state, const double &input, const uint64_t &dt) const;

  Eigen::VectorXd m_numerator, m_denominator;
  uint64_t m_matrix_size;
  Eigen::VectorXd m_current_state;
  Eigen::MatrixXd m_A_matrix;
  Eigen::VectorXd m_B_vector;
  Eigen::RowVectorXd m_C_vector;
  double m_D;

  // Internal variables for RK4 calculations
  mutable Eigen::VectorXd m_k1, m_k2, m_k3, m_k4;

  std::vector<std::function<Eigen::VectorXd(const Eigen::VectorXd &, const double &, const uint64_t &)>> m_integrators;
};

}  // namespace control_toolbox
