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

#include "rnrt_control_tools/transfer_fcn.h"

namespace control_toolbox
{
TransferFcn::TransferFcn()
{
}

TransferFcn::TransferFcn(const std::vector<double> &num, const std::vector<double> &den)
  : m_numerator{ removeLeadingZeros(num) }, m_denominator{ removeLeadingZeros(den) }
{
  if (!isValid())
  {
    throw std::invalid_argument("Invalid transfer function input");
  }
}

TransferFcn::TransferFcn(const TransferFcn &tf)
{
  m_numerator = tf.m_numerator;
  m_denominator = tf.m_denominator;
}

void TransferFcn::setNumerator(const std::vector<double> &num)
{
  m_numerator = removeLeadingZeros(num);
}

void TransferFcn::setDenominator(const std::vector<double> &den)
{
  m_denominator = removeLeadingZeros(den);
}

void TransferFcn::init(const std::vector<double> &num, const std::vector<double> &den)
{
  setNumerator(num);
  setDenominator(den);
}


std::vector<double> TransferFcn::getNumerator() const
{
  return m_numerator;
}
std::vector<double> TransferFcn::getDenominator() const
{
  return m_denominator;
}

std::vector<double> TransferFcn::removeLeadingZeros(const std::vector<double> &input) const
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

bool TransferFcn::isValid() const
{
  if (m_numerator.empty() || m_denominator.empty())
  {
    return false;
  }

  return true;
}

bool TransferFcn::isProper() const
{
  if (m_numerator.size() > m_denominator.size())
  {
    return false;
  }
  return true;
}

}  // namespace control_toolbox
