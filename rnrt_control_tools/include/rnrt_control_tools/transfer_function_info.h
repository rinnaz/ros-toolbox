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
  Desc: Implements a linear dynamic system (ODE) transfer function description
*/

#pragma once

#include <exception>
#include <iostream>
#include <limits>
#include <vector>

namespace control_toolbox
{
class TransferFunctionInfo
{
public:
  // Default constructor
  TransferFunctionInfo();

  TransferFunctionInfo(const std::vector<double>& numerator, const std::vector<double>& denominator);
  
  // Copy constructor
  TransferFunctionInfo(const TransferFunctionInfo& tf);

  ~TransferFunctionInfo(){};

  void init(const std::vector<double>& numerator, const std::vector<double>& denominator);

  // For numerator defined as b_n*s^n + ... + b_1*s + b_0
  // vector values should be {b_n, ..., b_1, b_0}
  void setNumerator(const std::vector<double>& numerator);

  // For denominator defined as a_n*s^n + ... + a_1*s + a_0
  // vector values should be {a_n, ..., a_1, a_0}
  void setDenominator(const std::vector<double>& denominator);

  // Check if transfer function is proper, whick means that
  // numerator size is less or equal to denominator size
  bool isProper() const;

  // Check that numerator and denominator are not empty vectors
  bool isValid() const;

  std::vector<double> getNumerator() const;
  std::vector<double> getDenominator() const;

protected:
  std::vector<double> m_numerator;
  std::vector<double> m_denominator;
  
  // Check if highest order values are zeros and remove them if they are
  std::vector<double> removeLeadingZeros(const std::vector<double>&) const;
};

}  // namespace control_toolbox
