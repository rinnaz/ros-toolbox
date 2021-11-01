#include <limits>
#include <iostream>
#include "rnrt_control_tools/transfer_fcn.h"
#include "rnrt_control_tools/state_space_model.h"
#include "gtest/gtest.h"

// TEST(StateSpaceModelTest, ConstructorTest)
// {
//     RecordProperty(
//         "description",
//         "This test check if Constructor works fine");

//     TransferFcn tfcn{{1.0}, {0.05, 1.0}};

//     EXPECT_NO_THROW(StateSpaceModel ssm{tfcn});
// }

TEST(StateSpaceModelTest, EulerTest)
{
    RecordProperty(
        "description",
        "This test check if Euler solver works fine");

    TransferFcn tfcn{{1.0}, {0.05, 1.0}};
    StateSpaceModel ssm{tfcn};

    double value{0.0};

    uint64_t msec = 1 * 1e6;

    for (auto i{0}; i < 100; i++)
    {
        value = ssm.getResponse(1.0, 10 * msec, SolverType::EULER);
        // std::cerr << value <<std::endl;
    }

    EXPECT_TRUE((value - 1.0) < 1.0e-5);
}

// TEST(StateSpaceModelTest, RungeKuttaTest)
// {
//     RecordProperty(
//         "description",
//         "This test check if RungeKutta solver works fine");

//     TransferFcn tfcn{{1.0}, {0.05, 1.0}};
//     StateSpaceModel ssm{tfcn};

//     double value{0.0};

//     uint64_t msec = 1 * 1e6;

//     for (auto i{0}; i < 10; i++)
//     {
//         value = ssm.getResponse(1.0, 10 * msec, SolverType::RUNGEKUTTA);
//     }

//     EXPECT_TRUE((value - 1.0) < 1.0e-5);

//     // EXPECT_DOUBLE_EQ(value, 1.0);
// }

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}