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

#include "rnrt_control_tools/solvers.h"

namespace control_toolbox
{
VectorXdL EulerSolver::integrate(const StateSpaceModel& model, const VectorXdL& last_state, const double& input,
                                 const uint64_t& dt)
{
  return last_state + dt / 1e9 * model.computeDerivatives(last_state, input);
}

VectorXdL RK4Solver::integrate(const StateSpaceModel& model, const VectorXdL& last_state, const double& input,
                               const uint64_t& dt)
{
  k1_ = dt / 1e9 * model.computeDerivatives(last_state, input);
  k2_ = dt / 1e9 * model.computeDerivatives(last_state + k1_ / 2.0, input);
  k3_ = dt / 1e9 * model.computeDerivatives(last_state + k2_ / 2.0, input);
  k4_ = dt / 1e9 * model.computeDerivatives(last_state + k3_, input);

  return last_state + (k1_ + 2.0 * k2_ + 2.0 * k3_ + k4_) / 6;
}

std::unique_ptr<SolverInterface> SolverFactory::createSolver(const SolverType& solver)
{
  switch (solver)
  {
    case SolverType::EULER:
      return std::unique_ptr<SolverInterface>(new EulerSolver());
    case SolverType::RK4:
      return std::unique_ptr<SolverInterface>(new RK4Solver());
  }
  return std::unique_ptr<SolverInterface>(new EulerSolver());
}

}  // namespace control_toolbox
