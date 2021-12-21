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
const uint64_t eigen_matrix_size_limit{ 20 }; // supposedly you don't need a system with order higher than 20
using MatrixXdL = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::AutoAlign, eigen_matrix_size_limit,
                                eigen_matrix_size_limit>;
using VectorXdL = Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::AutoAlign, eigen_matrix_size_limit, 1>;

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
   * \brief Compute new state vector and system output with nonuniform
   *        time step size
   *
   * \param last_state  State vector from previous step
   * \param input  Model input
   * \param dt  Time step in nanoseconds
   * \param solver  ODE solver type
   *
   * \returns System response
   */
  double computeResponse(const VectorXdL &last_state, const double &input, const uint64_t &dt,
                         const SolverType = SolverType::EULER);

  /*!
   * \brief Compute system output with nonuniform
   *        time step size
   *
   * \param input  Model input
   * \param dt  Time step in nanoseconds
   * \param solver  ODE solver type
   *
   * \returns System response
   */
  double computeResponse(const double &input, const uint64_t &dt, const SolverType = SolverType::EULER);

  /*!
   * \brief Resets current state vector
   */
  void resetState();

  MatrixXdL getMatrixA();
  MatrixXdL getMatrixB();

private:
  VectorXdL makeNumerator(const TransferFunctionInfo &tfcn) const;
  VectorXdL makeDenominator(const TransferFunctionInfo &tfcn) const;
  MatrixXdL calcMatrixA() const;
  MatrixXdL calcMatrixB() const;
  MatrixXdL calcMatrixC() const;
  MatrixXdL calcMatrixD() const;
  VectorXdL computeDerivatives(const VectorXdL &state, const double &input) const;

  VectorXdL integrateEuler(const VectorXdL &last_state, const double &input, const uint64_t &dt) const;

  VectorXdL integrateRK4(const VectorXdL &last_state, const double &input, const uint64_t &dt) const;

  VectorXdL numerator_, denominator_;
  uint64_t matrix_size_;
  VectorXdL current_state_;
  MatrixXdL A_matrix_;
  MatrixXdL B_matrix_;
  MatrixXdL C_matrix_;
  MatrixXdL D_matrix_;

  // Internal variables for RK4 calculations
  mutable VectorXdL k1_, k2_, k3_, k4_;

  std::vector<std::function<VectorXdL(const VectorXdL &, const double &, const uint64_t &)>> integrators_;
};

}  // namespace control_toolbox
