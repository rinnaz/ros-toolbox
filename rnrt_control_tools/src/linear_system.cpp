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
  Desc: Implements a wrapper for state space model and transfer function info
*/

#include "rnrt_control_tools/linear_system.h"

namespace control_toolbox
{
LinearSystem::LinearSystem()
{
}

LinearSystem::LinearSystem(const std::vector<double>& numerator, const std::vector<double>& denominator,
                           const SolverType solver)
{
  init(numerator, denominator, solver);
}

void LinearSystem::init(const std::vector<double>& numerator, const std::vector<double>& denominator,
                        const SolverType solver)
{
  solver_ = solver;
  tfcn_ = std::make_shared<TransferFunctionInfo>(numerator, denominator);

  model_ = std::make_shared<StateSpaceModel>();

  model_->init(*tfcn_, solver);
}

bool LinearSystem::init(const ros::NodeHandle& n, const SolverType solver)
{
  ros::NodeHandle nh(n);
  std::vector<double> num, den;

  // Load system parameters from parameter server
  if (!nh.getParam("numerator", num))
  {
    ROS_ERROR("No numerator specified for transfer function.  Namespace: %s", nh.getNamespace().c_str());
    return false;
  }

  if (!nh.getParam("denominator", den))
  {
    ROS_ERROR("No denominator specified for transfer function.  Namespace: %s", nh.getNamespace().c_str());
    return false;
  }

  init(num, den, solver);

  return true;
}

double LinearSystem::computeResponse(const double& input, const uint64_t& time_step)
{
  return model_->computeResponse(input, time_step);
}

void LinearSystem::reset()
{
  model_->resetState();
}

}  // namespace control_toolbox
