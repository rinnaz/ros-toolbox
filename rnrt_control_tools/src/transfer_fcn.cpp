#include "rnrt_control_tools/transfer_fcn.h"

TransferFcn::TransferFcn()
{
}

TransferFcn::TransferFcn(const std::vector<double> &num, const std::vector<double> &den)
    : m_numerator{num},
      m_denominator{den}
{
    try
    {
        checkSizes();
        normalize();
    }
    catch (const std::invalid_argument &e)
    {
        std::cerr << "exception: " << e.what() << std::endl;
    }
}

void TransferFcn::checkSizes()
{
    if (m_numerator.size() > m_denominator.size() - 1)
    {
        throw std::invalid_argument("Numerator is greater than denominator");
    }
}

void TransferFcn::setNumerator(const std::vector<double> &num)
{
    m_numerator = num;
}

void TransferFcn::setDenominator(const std::vector<double> &den)
{
    m_denominator = den;
}

std::vector<double> TransferFcn::getNumerator() const
{
    return m_numerator;
}
std::vector<double> TransferFcn::getDenominator() const
{
    return m_denominator;
}

void TransferFcn::normalize()
{
    double scaler {m_denominator.at(0)};

    for (auto i : m_denominator)
    {
        i = i / scaler;
    }

    for (auto i : m_numerator)
    {
        i = i / scaler;
    }
}