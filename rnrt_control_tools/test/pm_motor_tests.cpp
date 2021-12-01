#include <cmath>
#include <iostream>

#include "control_toolbox/pid.h"
#include "gtest/gtest.h"
#include "rnrt_control_tools/pm_motor.h"
#include "rnrt_control_tools/state_space_model.h"
#include "rnrt_control_tools/transfer_fcn.h"

using namespace control_toolbox;

TEST(PmMotorTest, ConstructorTest)
{
  RecordProperty("description", "Check if Constructor throws exceptions on incorrect input");

  EXPECT_NO_THROW(PmMotor motor0(1.0, 1.0, 1.0));

  EXPECT_THROW(PmMotor motor1(1.0, 1.0, -1.0), std::range_error);

  EXPECT_THROW(PmMotor motor2(1.0, -1.0, 1.0), std::range_error);

  EXPECT_THROW(PmMotor motor3(-1.0, 1.0, 1.0), std::range_error);

  EXPECT_THROW(PmMotor motor4(1.0, 1.0, 1.0, 0), std::range_error);
}

TEST(PmMotorTest, InitTest)
{
  RecordProperty("description", "Check if init() throws exception on incorrect input");

  PmMotor motor;

  EXPECT_THROW(motor.init(1.0, 1.0, 1.0, 0), std::range_error);

  EXPECT_THROW(motor.init(-1.0, 1.0, 1.0, 1), std::range_error);

  EXPECT_THROW(motor.init(1.0, -1.0, 1.0, 1), std::range_error);

  EXPECT_THROW(motor.init(1.0, 1.0, -1.0, 1), std::range_error);
}

TEST(PmMotorTest, RunTest)
{
  RecordProperty("description", "Check if motor returns correct response");

  double ind{ 0.001 };
  double res{ 0.8 };
  double km{ 1.5 };

  PmMotor motor;

  motor.init(ind, res, km, 1, SolverType::RUNGEKUTTA);

  auto result = motor.getCurrentResponse(48.0, 0.0, 1000000);
  EXPECT_DOUBLE_EQ(result, 32.896);

  result = motor.getCurrentResponse(48.0, 0.0, 1000000);
  EXPECT_DOUBLE_EQ(result, 47.756219733333330);

  result = motor.getCurrentResponse(48.0, 0.0, 1000000);
  EXPECT_DOUBLE_EQ(result, 54.469076327537780);

  // Check if reset works
  motor.reset();

  result = motor.getCurrentResponse(48.0, 0.0, 1000000);
  EXPECT_DOUBLE_EQ(result, 32.896);

  result = motor.getCurrentResponse(48.0, 0.0, 1000000);
  EXPECT_DOUBLE_EQ(result, 47.756219733333330);

  result = motor.getCurrentResponse(48.0, 0.0, 1000000);
  EXPECT_DOUBLE_EQ(result, 54.469076327537780);

  // Check if reset works
  motor.reset();

  result = motor.getCurrentResponse(48.0, 10.0, 1000000);
  EXPECT_DOUBLE_EQ(result, 22.615999999999996);

  result = motor.getCurrentResponse(48.0, 10.0, 1000000);
  EXPECT_DOUBLE_EQ(result, 32.832401066666660);

  result = motor.getCurrentResponse(48.0, 10.0, 1000000);
  EXPECT_DOUBLE_EQ(result, 37.447489975182220);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
