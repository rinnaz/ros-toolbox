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
  Desc: Implements a linear dynamic system (ODE) transfer function description
*/

#include "rnrt_control_tools/transfer_function_info.h"

namespace control_toolbox
{
TransferFunctionInfo::TransferFunctionInfo()
{
}

TransferFunctionInfo::TransferFunctionInfo(const std::vector<double> &num, const std::vector<double> &den)
  : numerator_{ removeLeadingZeros(num) }, denominator_{ removeLeadingZeros(den) }
{
  if (!isValid())
  {
    throw std::invalid_argument("Invalid transfer function input");
  }
}

TransferFunctionInfo::TransferFunctionInfo(const TransferFunctionInfo &tf)
{
  numerator_ = tf.numerator_;
  denominator_ = tf.denominator_;
}

void TransferFunctionInfo::setNumerator(const std::vector<double> &num)
{
  numerator_ = removeLeadingZeros(num);
}

void TransferFunctionInfo::setDenominator(const std::vector<double> &den)
{
  denominator_ = removeLeadingZeros(den);
}

void TransferFunctionInfo::init(const std::vector<double> &num, const std::vector<double> &den)
{
  setNumerator(num);
  setDenominator(den);
  if (!isValid())
  {
    throw std::invalid_argument("Invalid transfer function input");
  }
}

std::vector<double> TransferFunctionInfo::getNumerator() const
{
  return numerator_;
}
std::vector<double> TransferFunctionInfo::getDenominator() const
{
  return denominator_;
}

std::vector<double> TransferFunctionInfo::removeLeadingZeros(const std::vector<double> &input) const
{
  std::vector<double> result;

  for (auto i = input.begin(); i != input.end(); i++)
  {
    if (*i != 0.0 || std::abs(*i) > std::numeric_limits<double>::epsilon())
    {
      result = std::vector<double>(i, input.end());
      break;
    }
  }
  return result;
}

bool TransferFunctionInfo::isValid() const
{
  if (numerator_.empty() || denominator_.empty())
  {
    return false;
  }

  return true;
}

bool TransferFunctionInfo::isProper() const
{
  if (numerator_.size() > denominator_.size())
  {
    return false;
  }
  return true;
}

}  // namespace control_toolbox
