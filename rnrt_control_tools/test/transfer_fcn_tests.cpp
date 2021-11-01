#include <limits>
#include <iostream>
#include "rnrt_control_tools/transfer_fcn.h"
#include "gtest/gtest.h"

TEST(TransferFcnTest, ValidationTest)
{
    RecordProperty(
        "description",
        "This test check if Transfer function validation works fine");

    TransferFcn tfcn{{1.0, 1.0}, {10.0, 1.0}};
    EXPECT_TRUE(tfcn.isValid());

    tfcn = TransferFcn({{0.0, 1.0, 1.0}, {10.0, 1.0}});
    EXPECT_TRUE(tfcn.isValid());

    tfcn = TransferFcn({{1.0, 1.0, 1.0}, {10.0, 1.0}});
    EXPECT_TRUE(tfcn.isValid());

    tfcn = TransferFcn({{1.0, 1.0}, {0.0, 1.0}});
    EXPECT_TRUE(tfcn.isValid());

    tfcn = TransferFcn({{0.0, 0.0}, {0.0, 1.0}});
    EXPECT_FALSE(tfcn.isValid());

    tfcn = TransferFcn({{0.0, 0.0}, {0.0, 0.0}});
    EXPECT_FALSE(tfcn.isValid());

    tfcn = TransferFcn({{1.0, 0.0}, {0.0, 0.0}});
    EXPECT_FALSE(tfcn.isValid());

    tfcn = TransferFcn({{}, {1.0}});
    EXPECT_FALSE(tfcn.isValid());

    tfcn = TransferFcn({{1.0}, {}});
    EXPECT_FALSE(tfcn.isValid());

    tfcn = TransferFcn({{}, {}});
    EXPECT_FALSE(tfcn.isValid());
}

TEST(TransferFcnTest, ProperTest)
{
    RecordProperty(
        "description",
        "This test check if Transfer function proper check works fine");

    TransferFcn tfcn{{1.0, 1.0}, {10.0, 1.0}};
    EXPECT_TRUE(tfcn.isProper());

    tfcn = TransferFcn({{0.0, 1.0, 1.0}, {10.0, 1.0}});
    EXPECT_TRUE(tfcn.isProper());

    tfcn = TransferFcn({{1.0, 1.0, 1.0}, {10.0, 1.0}});
    EXPECT_FALSE(tfcn.isProper());

    tfcn = TransferFcn({{1.0, 1.0}, {0.0, 1.0}});
    EXPECT_FALSE(tfcn.isProper());
}


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}