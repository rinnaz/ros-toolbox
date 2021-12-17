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

#pragma once

#include <ros/ros.h>

#include <memory>
#include <vector>

#include "rnrt_control_tools/state_space_model.h"
#include "rnrt_control_tools/transfer_function_info.h"

namespace control_toolbox
{
/*!
 * \brief Represents continuous domain linear dynamical system
 *
 *      Contains transfer function description and its state space representation,
 *      takes input and returns system response
 */
class LinearSystem
{
public:
  LinearSystem();

  /*!
   * \brief Constructor, initializes transfer function description and state space model
   *        from numerator and denominator coefficients
   *
   * \param numerator  Transfer function numerator
   * \param denominator  Transfer function denominator
   * \param solver  ODE solver type
   */
  LinearSystem(const std::vector<double> &numerator, const std::vector<double> &denominator,
               const SolverType solver = SolverType::EULER);
  ~LinearSystem(){};

  /*!
   * \brief Initializes transfer function description and state space model
   *        from numerator and denominator coefficients
   *
   * \param numerator  Transfer function numerator
   * \param denominator  Transfer function denominator
   * \param solver  ODE solver type
   */
  void init(const std::vector<double> &numerator, const std::vector<double> &denominator,
            const SolverType solver = SolverType::EULER);

  /*!
   * \brief Initializes transfer function description and state space model
   *        from ROS parameter server
   *
   * \param n  The NodeHandle which should be used to query parameters
   * \param solver  ODE solver type
   */
  bool init(const ros::NodeHandle &n, const SolverType solver = SolverType::EULER);

  /*!
   * \brief Computes system response based on input with nonuniform
   *        time step size
   *
   * \param input  System input
   * \param time_step  Time step, change in time since last call
   *
   * \returns System response
   */
  double computeResponse(const double &input, const uint64_t &time_step);

  void reset();

protected:
  SolverType solver_;
  std::shared_ptr<TransferFunctionInfo> tfcn_;
  std::shared_ptr<StateSpaceModel> model_;
};

using Filter = LinearSystem;

}  // namespace control_toolbox
