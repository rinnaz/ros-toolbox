#include <cmath>
#include <iostream>
#include "rnrt_control_tools/transfer_fcn.h"
#include "rnrt_control_tools/state_space_model.h"
#include "rnrt_control_tools/pm_motor.h"
#include "gtest/gtest.h"

TEST(PmMotorTest, ConstructorTest)
{
    RecordProperty(
        "description",
        "This test check if Constructor throws exceptions on incorrect input");

    EXPECT_NO_THROW(PmMotor motor0(1.0, 1.0, 1.0));

    EXPECT_THROW(PmMotor motor1(1.0, 1.0, -1.0), std::range_error);

    EXPECT_THROW(PmMotor motor2(1.0, -1.0, 1.0), std::range_error);

    EXPECT_THROW(PmMotor motor3(-1.0, 1.0, 1.0), std::range_error);

    EXPECT_THROW(PmMotor motor4(1.0, 1.0, 1.0, 0), std::range_error);
}

TEST(PmMotorTest, InitTest)
{
    RecordProperty(
        "description",
        "This test check if init() throws exception on incorrect input");

    PmMotor motor;

    EXPECT_THROW(motor.init(1.0, 1.0, 1.0, 0), std::range_error);

    EXPECT_THROW(motor.init(-1.0, 1.0, 1.0, 1), std::range_error);

    EXPECT_THROW(motor.init(1.0, -1.0, 1.0, 1), std::range_error);

    EXPECT_THROW(motor.setTe(1.0, 1.0, -1.0, 1), std::range_error);

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
