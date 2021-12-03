#include <cmath>
#include <iostream>
#include <memory>

#include "control_toolbox/pid.h"
#include "gtest/gtest.h"
#include "rnrt_control_tools/linear_system.h"
#include "rnrt_control_tools/state_space_model.h"
#include "rnrt_control_tools/transfer_function_info.h"

using namespace control_toolbox;

TEST(LinearSystemTest, ConstructorTest)
{
  RecordProperty("description", "Check if Constructor throws exceptions on incorrect input");

  std::shared_ptr<LinearSystem> linsys_ptr;

  EXPECT_NO_THROW(linsys_ptr =
                      std::make_shared<LinearSystem>(std::vector<double>({ 1.0 }), std::vector<double>({ 10.0, 2.0 })));

  EXPECT_NO_THROW(linsys_ptr = std::make_shared<LinearSystem>(std::vector<double>({ 1.0, 0.0 }),
                                                              std::vector<double>({ 10.0, 2.0 })));

  EXPECT_THROW(linsys_ptr = std::make_shared<LinearSystem>(std::vector<double>({ 1.0, 1.0, 1.0 }),
                                                           std::vector<double>({ 10.0, 2.0 })),
               std::invalid_argument);

  EXPECT_THROW(linsys_ptr = std::make_shared<LinearSystem>(std::vector<double>({ 1.0, 1.0, 1.0 }),
                                                           std::vector<double>({ 10.0, 2.0 })),
               std::invalid_argument);

  EXPECT_THROW(linsys_ptr = std::make_shared<LinearSystem>(std::vector<double>({ 1.0, 1.0, 1.0 }),
                                                           std::vector<double>({ 0.0, 10.0, 2.0 })),
               std::invalid_argument);

  EXPECT_THROW(linsys_ptr = std::make_shared<LinearSystem>(std::vector<double>({ 1.0, 1.0, 1.0 }),
                                                           std::vector<double>({ 0.0, 0.0, 0.05, 1.0 })),
               std::invalid_argument);
}

TEST(LinearSystemTest, InitTest)
{
  RecordProperty("description", "Check if init() throws exception on incorrect input");

  LinearSystem linsys;

  EXPECT_NO_THROW(linsys.init({ 1.0 }, { 10.0, 2.0 }));

  EXPECT_NO_THROW(linsys.init({ 1.0, 0.0 }, { 10.0, 2.0 }));

  EXPECT_THROW(linsys.init({ 1.0, 1.0, 1.0 }, { 10.0, 2.0 }), std::invalid_argument);

  EXPECT_THROW(linsys.init({ 1.0, 1.0, 1.0 }, { 10.0, 2.0 }), std::invalid_argument);

  EXPECT_THROW(linsys.init({ 1.0, 1.0, 1.0 }, { 0.0, 10.0, 2.0 }), std::invalid_argument);

  EXPECT_THROW(linsys.init({ 1.0, 1.0, 1.0 }, { 0.0, 0.0, 0.05, 1.0 }), std::invalid_argument);
}

TEST(LinearSystemTest, EulerRunTest)
{
  RecordProperty("description", "Check if LinearSystem object returns correct response, using Euler solver");

  TransferFunctionInfo tfcn{ { 1.0 }, { 0.05, 1.0 } };

  LinearSystem linsys;
  linsys.init(tfcn.getNumerator(), tfcn.getDenominator(), SolverType::EULER);

  double value{ 0.0 };

  uint64_t msec = 1 * 1e6;

  for (auto i{ 0 }; i < 100; i++)
  {
    value = linsys.computeResponse(1.0, 10 * msec);
  }

  double epsilon{ 1.0e-8 };

  EXPECT_TRUE(abs(value - 1.0) < epsilon);
}

TEST(LinearSystemTest, RK4RunTest)
{
  RecordProperty("description", "Check if LinearSystem object returns correct response, using RK4 solver");

  TransferFunctionInfo tfcn{ { 1.0 }, { 0.05, 1.0 } };

  LinearSystem linsys;
  linsys.init(tfcn.getNumerator(), tfcn.getDenominator(), SolverType::RK4);

  double value{ 0.0 };

  uint64_t msec = 1 * 1e6;

  for (auto i{ 0 }; i < 100; i++)
  {
    value = linsys.computeResponse(1.0, 10 * msec);
  }

  double epsilon{ 1.0e-8 };

  EXPECT_TRUE(abs(value - 1.0) < epsilon);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
