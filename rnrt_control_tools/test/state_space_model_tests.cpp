#include <cmath>
#include <iostream>

#include "gtest/gtest.h"
#include "rnrt_control_tools/state_space_model.h"
#include "rnrt_control_tools/transfer_fcn.h"

using namespace control_toolbox;

TEST(StateSpaceModelTest, ConstructorTest)
{
  RecordProperty("description", "Check if Constructor throws on incorrect input");

  TransferFcn tfcn{ { 1.0 }, { 0.05, 1.0 } };

  EXPECT_NO_THROW(StateSpaceModel ssm0{ tfcn });

  tfcn = TransferFcn({ { 1.0, 1.0 }, { 0.05, 1.0 } });

  EXPECT_NO_THROW(StateSpaceModel ssm1{ tfcn });

  tfcn = TransferFcn({ { 1.0, 1.0, 1.0 }, { 0.05, 1.0 } });

  EXPECT_THROW(StateSpaceModel ssm2{ tfcn }, std::invalid_argument);

  tfcn = TransferFcn({ { 1.0, 1.0, 1.0 }, { 0.0, 0.05, 1.0 } });

  EXPECT_THROW(StateSpaceModel ssm3{ tfcn }, std::invalid_argument);

  tfcn = TransferFcn({ { 1.0, 1.0, 1.0 }, { 0.0, 0.0, 0.05, 1.0 } });

  EXPECT_THROW(StateSpaceModel ssm4{ tfcn }, std::invalid_argument);
}

TEST(StateSpaceModelTest, InitTest)
{
  RecordProperty("description", "Check if initializer throws on incorrect input");

  TransferFcn tfcn{ { 1.0 }, { 0.05, 1.0 } };
  StateSpaceModel ssm;
  EXPECT_NO_THROW(ssm.init(tfcn));
  EXPECT_NO_THROW(ssm.init({ 1.0 }, { 0.05, 1.0 }));

  tfcn = TransferFcn({ { 1.0, 1.0 }, { 0.05, 1.0 } });

  EXPECT_NO_THROW(ssm.init(tfcn));
  EXPECT_NO_THROW(ssm.init({ 1.0, 1.0 }, { 0.05, 1.0 }));

  tfcn = TransferFcn({ { 1.0, 1.0, 1.0 }, { 0.05, 1.0 } });

  EXPECT_THROW(ssm.init(tfcn), std::invalid_argument);
  EXPECT_THROW(ssm.init({ 1.0, 1.0, 1.0 }, { 0.05, 1.0 }), std::invalid_argument);

  tfcn = TransferFcn({ { 1.0, 1.0, 1.0 }, { 0.0, 0.05, 1.0 } });

  EXPECT_THROW(ssm.init(tfcn), std::invalid_argument);
  EXPECT_THROW(ssm.init({ 1.0, 1.0, 1.0 }, { 0.0, 0.05, 1.0 }), std::invalid_argument);

  tfcn = TransferFcn({ { 1.0, 1.0, 1.0 }, { 0.0, 0.0, 0.05, 1.0 } });

  EXPECT_THROW(ssm.init(tfcn), std::invalid_argument);
  EXPECT_THROW(ssm.init({ 1.0, 1.0, 1.0 }, { 0.0, 0.0, 0.05, 1.0 }), std::invalid_argument);
}

TEST(StateSpaceModelTest, EulerTest)
{
  RecordProperty("description", "Check if Euler solver works fine");

  TransferFcn tfcn{ { 1.0 }, { 0.05, 1.0 } };
  StateSpaceModel ssm{ tfcn };

  double value{ 0.0 };

  uint64_t msec = 1 * 1e6;

  for (auto i{ 0 }; i < 100; i++)
  {
    value = ssm.computeResponse(1.0, 10 * msec, SolverType::EULER);
  }

  double epsilon{ 1.0e-8 };

  EXPECT_TRUE(abs(value - 1.0) < epsilon);
}

TEST(StateSpaceModelTest, RungeKuttaTest)
{
  RecordProperty("description", "Check if RungeKutta solver works fine");

  TransferFcn tfcn{ { 1.0 }, { 0.05, 1.0 } };
  StateSpaceModel ssm{ tfcn };

  double value{ 0.0 };

  uint64_t msec = 1 * 1e6;

  for (auto i{ 0 }; i < 100; i++)
  {
    value = ssm.computeResponse(1.0, 10 * msec, SolverType::RUNGEKUTTA);
  }

  double epsilon{ 1.0e-8 };

  EXPECT_TRUE(abs(value - 1.0) < epsilon);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}