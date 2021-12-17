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

#pragma once

#include <exception>
#include <iostream>
#include <limits>
#include <vector>

namespace control_toolbox
{
/*!
 * \brief Describes transfer function of linear dynamical system
 *
 *    Transfer function is defined with numerator and denominator
 *            b_n*s^n + ... + b_1*s + b_0
 *          ---------------------------------
 *            a_n*s^n + ... + a_1*s + a_0
 *
 */
class TransferFunctionInfo
{
public:
  // Default constructor
  TransferFunctionInfo();

  /*!
   * \brief Constructor, initializes numerator and denominator of transfer function
   *
   * \param numerator  Vector of numerator polinomial coefficients
   * \param denominator  Vector of denominator polinomial coefficients
   */
  TransferFunctionInfo(const std::vector<double>& numerator, const std::vector<double>& denominator);

  /*!
   * \brief Copy Constructor
   *
   * \param tfcn  Transfer function to make copy of
   */
  TransferFunctionInfo(const TransferFunctionInfo& tfcn);

  ~TransferFunctionInfo(){};

  /*!
   * \brief Constructor, initializes numerator and denominator of transfer function
   *
   * \param numerator  Vector of numerator polinomial coefficients
   * \param denominator  Vector of denominator polinomial coefficients
   */
  void init(const std::vector<double>& numerator, const std::vector<double>& denominator);

  /*!
   * \brief Sets numerator
   *
   *    For numerator defined as b_n*s^n + ... + b_1*s + b_0
   *    vector values should be {b_n, ..., b_1, b_0}
   *
   * \param numerator  Vector of numerator polinomial coefficients
   */
  void setNumerator(const std::vector<double>& numerator);

  /*!
   * \brief Sets numerator
   *
   *    For denominator defined as a_n*s^n + ... + a_1*s + a_0
   *    vector values should be {a_n, ..., a_1, a_0}
   *
   * \param denominator  Vector of denominator polinomial coefficients
   */
  void setDenominator(const std::vector<double>& denominator);

  /*!
   * \brief Check if transfer function is proper
   *
   *    Which means that numerator size is less or equal to denominator size
   */
  bool isProper() const;

  /*!
   * \brief Check that numerator and denominator are not empty vectors
   */
  // Check that numerator and denominator are not empty vectors
  bool isValid() const;

  /*!
   * \brief Returns vector of numerator polinomial coefficients
   */
  std::vector<double> getNumerator() const;

  /*!
   * \brief Returns vector of denominator polinomial coefficients
   */
  std::vector<double> getDenominator() const;

protected:
  std::vector<double> numerator_;
  std::vector<double> denominator_;

  /*!
   * \brief Removes highest order polinomial coefficients set by zeros
   */
  std::vector<double> removeLeadingZeros(const std::vector<double>&) const;
};

}  // namespace control_toolbox
