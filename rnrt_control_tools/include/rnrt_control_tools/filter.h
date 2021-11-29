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
  Desc: Implements a continuous Butterworth filter
*/

#pragma once

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

#include "rnrt_control_tools/linear_system.h"
#include "rnrt_control_tools/transfer_fcn.h"

namespace control_toolbox
{
// Base abstract class
class ButterworthFilterBase : public control_toolbox::Filter
{
public:
  ButterworthFilterBase(){ initTfcnSelector(); }

  virtual void init(const uint64_t &order, const double &cutoff_frequency,
                    const SolverType solver = SolverType::EULER) = 0;
  bool init(const ros::NodeHandle &n, const SolverType solver = SolverType::EULER);

protected:
  double m_order;
  double m_cutoff_frequency;
  std::map<uint64_t, std::vector<double>> m_tfcn_selector;

  void initTfcnSelector();

  virtual control_toolbox::TransferFcn constructTfcn() = 0;
};

// Low-pass filter
class ButterworthFilterLowPass : public ButterworthFilterBase
{
public:
  ButterworthFilterLowPass(){};
  ButterworthFilterLowPass(const uint64_t &order, const double &cutoff_frequency,
                           const SolverType solver = SolverType::EULER);
  ~ButterworthFilterLowPass(){};

  void init(const uint64_t &order, const double &cutoff_frequency,
            const SolverType solver = SolverType::EULER) override;

protected:
  control_toolbox::TransferFcn constructTfcn() override;
};

// High-pass filter
class ButterworthFilterHighPass : public ButterworthFilterBase
{
public:
  ButterworthFilterHighPass(){};
  ButterworthFilterHighPass(const uint64_t &order, const double &cutoff_frequency,
                            const SolverType solver = SolverType::EULER);
  ~ButterworthFilterHighPass(){};

  void init(const uint64_t &order, const double &cutoff_frequency,
            const SolverType solver = SolverType::EULER) override;

protected:
  control_toolbox::TransferFcn constructTfcn() override;
};

}  // namespace control_toolbox
