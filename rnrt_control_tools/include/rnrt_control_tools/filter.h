#pragma once

#include <ros/ros.h>

#include <cmath>
#include <map>
#include <memory>
#include <vector>

#include "rnrt_control_tools/linear_system.h"
#include "rnrt_control_tools/transfer_fcn.h"

namespace control_tools
{

class ButterworthFilter : public control_tools::Filter
{
public:
  ButterworthFilter(){};
  ButterworthFilter(const uint64_t &order, const double &cutoff_frequency, const SolverType solver = SolverType::EULER);
  ~ButterworthFilter(){};

  void init(const uint64_t &order, const double &cutoff_frequency, const SolverType solver = SolverType::EULER);
  bool init(const ros::NodeHandle &n, const SolverType solver = SolverType::EULER);

protected:
  double m_order;
  double m_cutoff_frequency;
  std::map<uint64_t, std::vector<double>> m_tfcn_selector;

  void initTfcnSelector();
  control_tools::TransferFcn constructTfcn();
};

}  // namespace control_tools
