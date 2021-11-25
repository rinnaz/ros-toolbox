#include "rnrt_control_tools/transfer_fcn.h"

namespace control_tools
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

}  // namespace control_tools
