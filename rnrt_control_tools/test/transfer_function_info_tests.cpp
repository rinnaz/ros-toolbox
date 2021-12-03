#include <iostream>
#include <limits>

#include "gtest/gtest.h"
#include "rnrt_control_tools/transfer_function_info.h"

using namespace control_toolbox;

TEST(TransferFunctionInfoTest, ConstructorTest)
{
  RecordProperty("description", "Check if Transfer function validation works fine");

  TransferFunctionInfo tfcn;

  EXPECT_NO_THROW(tfcn = TransferFunctionInfo({ { 1.0, 1.0 }, { 10.0, 1.0 } }));

  EXPECT_NO_THROW(tfcn = TransferFunctionInfo({ { 0.0, 1.0, 1.0 }, { 10.0, 1.0 } }));

  EXPECT_NO_THROW(tfcn = TransferFunctionInfo({ { 1.0, 1.0, 1.0 }, { 10.0, 1.0 } }));

  EXPECT_NO_THROW(tfcn = TransferFunctionInfo({ { 1.0, 1.0 }, { 0.0, 1.0 } }));

  EXPECT_THROW(tfcn = TransferFunctionInfo({ { 0.0, 0.0 }, { 0.0, 1.0 } }), std::invalid_argument);

  EXPECT_THROW(tfcn = TransferFunctionInfo({ { 0.0, 0.0 }, { 0.0, 0.0 } }), std::invalid_argument);

  EXPECT_THROW(tfcn = TransferFunctionInfo({ { 1.0, 0.0 }, { 0.0, 0.0 } }), std::invalid_argument);

  EXPECT_THROW(tfcn = TransferFunctionInfo({ {}, { 1.0 } }), std::invalid_argument);

  EXPECT_THROW(tfcn = TransferFunctionInfo({ { 1.0 }, {} }), std::invalid_argument);

  EXPECT_THROW(tfcn = TransferFunctionInfo({ {}, {} }), std::invalid_argument);
}

TEST(TransferFunctionInfoTest, IsProperTest)
{
  RecordProperty("description", "Check if Transfer function proper check works fine");

  TransferFunctionInfo tfcn{ { 1.0, 1.0 }, { 10.0, 1.0 } };
  EXPECT_TRUE(tfcn.isProper());

  tfcn = TransferFunctionInfo({ { 0.0, 1.0, 1.0 }, { 10.0, 1.0 } });
  EXPECT_TRUE(tfcn.isProper());

  tfcn = TransferFunctionInfo({ { 1.0, 1.0, 1.0 }, { 10.0, 1.0 } });
  EXPECT_FALSE(tfcn.isProper());

  tfcn = TransferFunctionInfo({ { 1.0, 1.0 }, { 0.0, 1.0 } });

  EXPECT_FALSE(tfcn.isProper());
}

TEST(TransferFunctionInfoTest, InitTest)
{
  RecordProperty("description", "Check if initialization is working correctly");

  TransferFunctionInfo tfcn;

  EXPECT_NO_THROW(tfcn.init({ 1.0, 1.0 }, { 10.0, 1.0 }));

  EXPECT_NO_THROW(tfcn.init({ 0.0, 1.0, 1.0 }, { 10.0, 1.0 }));

  EXPECT_NO_THROW(tfcn.init({ 1.0, 1.0, 1.0 }, { 10.0, 1.0 }));

  EXPECT_NO_THROW(tfcn.init({ 1.0, 1.0 }, { 0.0, 1.0 }));

  EXPECT_THROW(tfcn.init({ 0.0, 0.0 }, { 0.0, 1.0 }), std::invalid_argument);

  EXPECT_THROW(tfcn.init({ 0.0, 0.0 }, { 0.0, 0.0 }), std::invalid_argument);

  EXPECT_THROW(tfcn.init({ 1.0, 0.0 }, { 0.0, 0.0 }), std::invalid_argument);

  EXPECT_THROW(tfcn.init({}, { 1.0 }), std::invalid_argument);

  EXPECT_THROW(tfcn.init({ 1.0 }, {}), std::invalid_argument);

  EXPECT_THROW(tfcn.init({}, {}), std::invalid_argument);
}

TEST(TransferFunctionInfoTest, RemoveLeadingZeros)
{
  RecordProperty("description", "Check if leading zeros in numerator and denominator are removed");

  TransferFunctionInfo tfcn;
  tfcn.init({ 0.0, 1.0, 1.0 }, { 10.0, 1.0 });
  
  EXPECT_EQ(std::vector<double>({1.0, 1.0}), tfcn.getNumerator());

  tfcn.init({ 10.0, 1.0 }, { 0.0, 1.0, 1.0 });
  EXPECT_EQ(std::vector<double>({1.0, 1.0}), tfcn.getDenominator());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}