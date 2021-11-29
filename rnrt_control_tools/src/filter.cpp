#include "rnrt_control_tools/filter.h"

namespace control_tools
{
ButterworthFilter::ButterworthFilter(const uint64_t &order, const double &cutoff_frequency, const SolverType solver)
{
  init(order, cutoff_frequency, solver);
}

void ButterworthFilter::init(const uint64_t &order, const double &cutoff_frequency, const SolverType solver)
{
  if (order == 0)
  {
    throw std::invalid_argument("Filter order cannot be equal to zero");
  }

  if (cutoff_frequency <= 0.0)
  {
    throw std::invalid_argument("Filter cutoff_frequency cannot be less or equal to zero");
  }

  m_order = order;
  m_cutoff_frequency = cutoff_frequency;
  initTfcnSelector();
  control_tools::TransferFcn tfcn = constructTfcn();
  LinearSystem::init(tfcn.getNumerator(), tfcn.getDenominator(), solver);
}

bool ButterworthFilter::init(const ros::NodeHandle &n, const SolverType solver)
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

void ButterworthFilter::initTfcnSelector()
{
  m_tfcn_selector = { { 1, { 1.0, 1.0 } },
                      { 2, { 1.0, 1.4142, 1.0 } },
                      { 3, { 1.0, 2.0, 2.0, 1.0 } },
                      { 4, { 1.0, 2.6131, 3.4142, 2.6131, 1.0 } },
                      { 5, { 1.0, 3.2361, 5.2361, 5.2361, 3.2361, 1.0 } },
                      { 6, { 1.0, 3.8637, 7.4641, 9.1416, 7.4641, 3.8637, 1.0 } },
                      { 7, { 1.0, 4.4940, 10.0978, 14.5918, 14.5918, 10.0978, 4.4940, 1.0 } },
                      { 8, { 1.0, 5.1258, 13.1371, 21.8462, 25.6884, 21.8462, 13.1371, 5.1258, 1.0 } },
                      { 9, { 1.0, 5.7588, 16.5817, 31.1634, 41.9864, 41.9864, 31.1634, 16.5817, 5.7588, 1.0 } },
                      { 10,
                        { 1, 6.3925, 20.4317, 42.8021, 64.8824, 74.2334, 64.8824, 42.8021, 20.4317, 6.3925, 1.0 } } };
}

control_tools::TransferFcn ButterworthFilter::constructTfcn()
{
  std::vector<double> num{ pow(m_cutoff_frequency, m_order) };
  std::vector<double> den{ m_tfcn_selector.at(m_order) };

  for (auto i{ 1 }; i < den.size(); i++)
  {
    den[i] = den[i] * pow(m_cutoff_frequency, i);
  }

  control_tools::TransferFcn ret{ num, den };

  return ret;
}

}  // namespace control_tools
