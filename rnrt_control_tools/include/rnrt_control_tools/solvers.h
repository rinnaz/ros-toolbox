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
  Desc: Implements ODEs solvers
*/

#pragma once

#include <vector>
#include <memory>

#include "eigen3/Eigen/Core"
#include "rnrt_control_tools/transfer_function_info.h"
#include "rnrt_control_tools/state_space_model.h"

namespace control_toolbox
{

class SolverInterface
{
public:
  virtual VectorXdL integrate(const StateSpaceModel& model, const VectorXdL& last_state, const double& input,
                              const uint64_t& dt) = 0;
};

class EulerSolver : public SolverInterface
{
public:
  VectorXdL integrate(const StateSpaceModel& model, const VectorXdL& last_state, const double& input,
                      const uint64_t& dt) override;
};

class RK4Solver : public SolverInterface
{
public:
  VectorXdL integrate(const StateSpaceModel& model, const VectorXdL& last_state, const double& input,
                      const uint64_t& dt) override;

protected:
  VectorXdL k1_, k2_, k3_, k4_;
};

class SolverFactory
{
public:
  static std::unique_ptr<SolverInterface> createSolver(const SolverType& solver);
};

}  // namespace control_toolbox
