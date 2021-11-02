#include <cmath>
#include <iostream>
#include "rnrt_control_tools/transfer_fcn.h"
#include "rnrt_control_tools/state_space_model.h"
#include "gtest/gtest.h"

TEST(StateSpaceModelTest, ConstructorTest)
{
    RecordProperty(
        "description",
        "This test check if Constructor works fine");

    TransferFcn tfcn{{1.0}, 
                     {0.05, 1.0}};

    EXPECT_NO_THROW(StateSpaceModel ssm0{tfcn});

    tfcn = TransferFcn({{1.0, 1.0}, 
                        {0.05, 1.0}});

    EXPECT_NO_THROW(StateSpaceModel ssm1{tfcn});

    tfcn = TransferFcn({{1.0, 1.0, 1.0}, 
                        {0.05, 1.0}});

    EXPECT_THROW(StateSpaceModel ssm2{tfcn}, std::invalid_argument);

    tfcn = TransferFcn({{1.0, 1.0, 1.0}, 
                        {0.0, 0.05, 1.0}});

    EXPECT_THROW(StateSpaceModel ssm3{tfcn}, std::invalid_argument);

    tfcn = TransferFcn({{1.0, 1.0, 1.0}, 
                        {0.0, 0.0, 0.05, 1.0}});

    EXPECT_THROW(StateSpaceModel ssm4{tfcn}, std::invalid_argument);
}

TEST(StateSpaceModelTest, EulerTest)
{
    RecordProperty(
        "description",
        "This test check if Euler solver works fine");

    TransferFcn tfcn{{1.0}, 
                     {0.05, 1.0}};
    StateSpaceModel ssm{tfcn};

    double value{0.0};

    uint64_t msec = 1 * 1e6;

    for (auto i{0}; i < 100; i++)
    {
        value = ssm.getResponse(1.0, 10 * msec, SolverType::EULER);
    }

    double epsilon{1.0e-8};

    EXPECT_TRUE(abs(value - 1.0) < epsilon);
}

TEST(StateSpaceModelTest, RungeKuttaTest)
{
    RecordProperty(
        "description",
        "This test check if RungeKutta solver works fine");

    TransferFcn tfcn{{1.0}, 
                     {0.05, 1.0}};
    StateSpaceModel ssm{tfcn};

    double value{0.0};

    uint64_t msec = 1 * 1e6;

    for (auto i{0}; i < 100; i++)
    {
        value = ssm.getResponse(1.0, 10 * msec, SolverType::RUNGEKUTTA);
    }

    double epsilon{1.0e-8};

    EXPECT_TRUE(abs(value - 1.0) < epsilon);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}