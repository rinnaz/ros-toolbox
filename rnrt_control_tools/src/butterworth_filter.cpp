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
  Desc: Implements a continuous Butterworth filter
*/

#include "rnrt_control_tools/butterworth_filter.h"

namespace filters
{
// ************************** ButterworthFilterBase ****************************

bool ButterworthFilterBase::init(const ros::NodeHandle &n, const SolverType solver)
{
  ros::NodeHandle nh(n);
  int order;
  double cutoff_frequency;

  // Load system parameters from parameter server
  if (!nh.getParam("order", order))
  {
    ROS_ERROR("No order specified for Butterworth Filter.  Namespace: %s", nh.getNamespace().c_str());
    return false;
  }

  if (order <= 0)
  {
    ROS_ERROR("Filter order cannot be less or equal to zero.  Namespace: %s", nh.getNamespace().c_str());
    return false;
  }

  if (!nh.getParam("cutoff_frequency", cutoff_frequency))
  {
    ROS_ERROR("No cutoff frequency specified for Butterworth Filter.  Namespace: %s", nh.getNamespace().c_str());
    return false;
  }

  init(order, cutoff_frequency, solver);

  return true;
}

void ButterworthFilterBase::initTransferFunctionSelector()
{
  tfcn_selector_ = {
    { 1, { 1.0, 1.0 } },
    { 2, { 1.0, 1.414213562373095, 1.0 } },
    { 3, { 1.0, 2.0, 2.0, 1.0 } },
    { 4, { 1.0, 2.613125929752753, 3.414213562373095, 2.613125929752753, 1.0 } },
    { 5, { 1.0, 3.2360679774997894, 5.236067977499789, 5.236067977499789, 3.2360679774997894, 1.0 } },
    { 6, { 1.0, 3.863703305156273, 7.4641016151377535, 9.14162017268564, 7.4641016151377535, 3.863703305156273, 1.0 } },
    { 7,
      { 1.0, 4.4939592074349335, 10.09783467904461, 14.591793886479543, 14.591793886479543, 10.09783467904461,
        4.4939592074349335, 1.0 } },
    { 8,
      { 1.0, 5.125830895483012, 13.137071184544087, 21.846150969207624, 25.688355931461274, 21.846150969207624,
        13.137071184544087, 5.125830895483012, 1.0 } },
    { 9,
      { 1.0, 5.758770483143633, 16.581718738763175, 31.16343747752635, 41.986385733145895, 41.986385733145895,
        31.16343747752635, 16.581718738763175, 5.758770483143633, 1.0 } },
    { 10,
      { 1.0, 6.39245322149966, 20.431729094530695, 42.80206106885194, 64.88239627026175, 74.23342925707763,
        64.88239627026175, 42.80206106885194, 20.431729094530695, 6.39245322149966, 1.0 } }
  };
}

// ************************* ButterworthFilterLowPass **************************

ButterworthFilterLowPass::ButterworthFilterLowPass(const uint64_t &order, const double &cutoff_frequency,
                                                   const SolverType solver)
{
  init(order, cutoff_frequency, solver);
}

void ButterworthFilterLowPass::init(const uint64_t &order, const double &cutoff_frequency, const SolverType solver)
{
  if (order == 0)
  {
    throw std::invalid_argument("Filter order cannot be equal to zero");
  }

  if (cutoff_frequency <= 0.0)
  {
    throw std::invalid_argument("Filter cutoff_frequency cannot be less or equal to zero");
  }

  // Limit order to range from 1 to 10
  order_ = std::clamp(order, uint64_t(1), uint64_t(10));

  cutoff_frequency_ = cutoff_frequency;

  TransferFunctionInfo tfcn = constructTransferFunction();
  LinearSystem::init(tfcn.getNumerator(), tfcn.getDenominator(), solver);
}

TransferFunctionInfo ButterworthFilterLowPass::constructTransferFunction()
{
  std::vector<double> num{ pow(cutoff_frequency_, order_) };
  std::vector<double> den{ tfcn_selector_.at(order_) };

  for (auto i{ 1 }; i < den.size(); i++)
  {
    den[i] = den[i] * pow(cutoff_frequency_, i);
  }

  return TransferFunctionInfo(num, den);
}

// ************************* ButterworthFilterHighPass *************************

ButterworthFilterHighPass::ButterworthFilterHighPass(const uint64_t &order, const double &cutoff_frequency,
                                                     const SolverType solver)
{
  init(order, cutoff_frequency, solver);
}

void ButterworthFilterHighPass::init(const uint64_t &order, const double &cutoff_frequency, const SolverType solver)
{
  if (order == 0)
  {
    throw std::invalid_argument("Filter order cannot be equal to zero");
  }

  if (cutoff_frequency <= 0.0)
  {
    throw std::invalid_argument("Filter cutoff_frequency cannot be less or equal to zero");
  }

  // Limit order to range from 1 to 10
  order_ = std::clamp(order, uint64_t(1), uint64_t(10));

  cutoff_frequency_ = cutoff_frequency;

  TransferFunctionInfo tfcn = constructTransferFunction();
  LinearSystem::init(tfcn.getNumerator(), tfcn.getDenominator(), solver);
}

TransferFunctionInfo ButterworthFilterHighPass::constructTransferFunction()
{
  std::vector<double> num(tfcn_selector_.at(order_).size(), 0.0);
  num[0] = 1.0;

  std::vector<double> den{ tfcn_selector_.at(order_) };

  std::reverse(den.begin(), den.end());

  for (auto i{ 1 }; i < den.size(); i++)
  {
    den[i] = den[i] * pow(cutoff_frequency_, i);
  }

  return TransferFunctionInfo(num, den);
}

}  // namespace filters
