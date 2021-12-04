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
#include <vector>

#include "eigen3/Eigen/Core"
#include "rnrt_control_tools/transfer_function_info.h"

namespace control_toolbox
{
enum class SolverType
{
  EULER = 0,
  RK4 = 1
};

/*!
 * \brief State space model representation of transfer function of continuous dynamical system
 *
 *    Transfer function is defined with numerator and denominator
 *            b_n*s^n + ... + b_1*s + b_0
 *          ---------------------------------
 *            a_n*s^n + ... + a_1*s + a_0
 *
 *    And state space model form is
 *                x' = A * x + B * u(t)
 *                y(t) = C * x + D * u(t)
 *    where "x" is state (vector), "u" is input (scalar), "y" is response (scalar)
 *    "A" is state matrix, "B" - input matrix, "C" - output matrix, D - feedforward matrix
 *
 */
class StateSpaceModel
{
public:
  StateSpaceModel();

  /*!
   * \brief Constructor, initializes internal parameters from TransferFunctionInfo object
   *
   * \param tfcn  Transfer function description to construct SSM with
   */
  StateSpaceModel(const TransferFunctionInfo &tfcn);

  /*!
   * \brief Constructor, initializes internal parameters from numerator and denominator coefficients
   *
   * \param numerator  Transfer function numerator
   * \param denominator  Transfer function denominator
   */
  StateSpaceModel(const std::vector<double> &numerator, const std::vector<double> &denominator);
  ~StateSpaceModel();

  /*!
   * \brief Initializes internal parameters from TransferFunctionInfo object
   *
   * \param tfcn  Transfer function description to construct SSM with
   */
  void init(const TransferFunctionInfo &tfcn);

  /*!
   * \brief Initializes internal parameters from numerator and denominator coefficients
   *
   * \param numerator  Transfer function numerator
   * \param denominator  Transfer function denominator
   */
  void init(const std::vector<double> &numerator, const std::vector<double> &denominator);

  /*!
   * \brief Compute new state vector
   *
   * \param last_state  State vector from previous step
   * \param input  Model input
   * \param dt  Time step in nanoseconds
   * \param solver  ODE solver type
   */
  double computeResponse(const Eigen::VectorXd &last_state, const double &input, const uint64_t &dt,
                         const SolverType = SolverType::EULER);

  /*!
   * \brief Compute system output
   *
   * \param input  Model input
   * \param dt  Time step in nanoseconds
   * \param solver  ODE solver type
   */
  double computeResponse(const double &input, const uint64_t &dt, const SolverType = SolverType::EULER);

  /*!
   * \brief Resets current state vector
   */
  void resetState();

  Eigen::MatrixXd getMatrixA();
  Eigen::MatrixXd getMatrixB();

private:
  Eigen::VectorXd makeNumerator(const TransferFunctionInfo &tfcn) const;
  Eigen::VectorXd makeDenominator(const TransferFunctionInfo &tfcn) const;
  Eigen::MatrixXd calcMatrixA() const;
  Eigen::MatrixXd calcMatrixB() const;
  Eigen::MatrixXd calcMatrixC() const;
  Eigen::MatrixXd calcMatrixD() const;
  Eigen::VectorXd computeDerivatives(const Eigen::VectorXd &state, const double &input) const;

  Eigen::VectorXd integrateEuler(const Eigen::VectorXd &last_state, const double &input, const uint64_t &dt) const;

  Eigen::VectorXd integrateRK4(const Eigen::VectorXd &last_state, const double &input, const uint64_t &dt) const;

  Eigen::VectorXd m_numerator, m_denominator;
  uint64_t m_matrix_size;
  Eigen::VectorXd m_current_state;
  Eigen::MatrixXd m_A_matrix;
  Eigen::MatrixXd m_B_matrix;
  Eigen::MatrixXd m_C_matrix;
  Eigen::MatrixXd m_D_matrix;

  // Internal variables for RK4 calculations
  mutable Eigen::VectorXd m_k1, m_k2, m_k3, m_k4;

  std::vector<std::function<Eigen::VectorXd(const Eigen::VectorXd &, const double &, const uint64_t &)>> m_integrators;
};

}  // namespace control_toolbox
