#include <iostream>
#include <limits>

#include "gtest/gtest.h"
#include "rnrt_control_tools/transfer_fcn.h"

using namespace control_toolbox;

TEST(TransferFcnTest, ConstructorTest)
{
  RecordProperty("description", "Check if Transfer function validation works fine");

  TransferFcn tfcn;

  EXPECT_NO_THROW(tfcn = TransferFcn({ { 1.0, 1.0 }, { 10.0, 1.0 } }));

  EXPECT_NO_THROW(tfcn = TransferFcn({ { 0.0, 1.0, 1.0 }, { 10.0, 1.0 } }));

  EXPECT_NO_THROW(tfcn = TransferFcn({ { 1.0, 1.0, 1.0 }, { 10.0, 1.0 } }));

  EXPECT_NO_THROW(tfcn = TransferFcn({ { 1.0, 1.0 }, { 0.0, 1.0 } }));

  EXPECT_THROW(tfcn = TransferFcn({ { 0.0, 0.0 }, { 0.0, 1.0 } }), std::invalid_argument);

  EXPECT_THROW(tfcn = TransferFcn({ { 0.0, 0.0 }, { 0.0, 0.0 } }), std::invalid_argument);

  EXPECT_THROW(tfcn = TransferFcn({ { 1.0, 0.0 }, { 0.0, 0.0 } }), std::invalid_argument);

  EXPECT_THROW(tfcn = TransferFcn({ {}, { 1.0 } }), std::invalid_argument);

  EXPECT_THROW(tfcn = TransferFcn({ { 1.0 }, {} }), std::invalid_argument);

  EXPECT_THROW(tfcn = TransferFcn({ {}, {} }), std::invalid_argument);
}

TEST(TransferFcnTest, IsProperTest)
{
  RecordProperty("description", "Check if Transfer function proper check works fine");

  TransferFcn tfcn{ { 1.0, 1.0 }, { 10.0, 1.0 } };
  EXPECT_TRUE(tfcn.isProper());

  tfcn = TransferFcn({ { 0.0, 1.0, 1.0 }, { 10.0, 1.0 } });
  EXPECT_TRUE(tfcn.isProper());

  tfcn = TransferFcn({ { 1.0, 1.0, 1.0 }, { 10.0, 1.0 } });
  EXPECT_FALSE(tfcn.isProper());

  tfcn = TransferFcn({ { 1.0, 1.0 }, { 0.0, 1.0 } });

  EXPECT_FALSE(tfcn.isProper());
}

TEST(TransferFcnTest, InitTest)
{
  RecordProperty("description", "Check if initialization is working correctly");

  TransferFcn tfcn;

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

TEST(TransferFcnTest, RemoveLeadingZeros)
{
  RecordProperty("description", "Check if leading zeros in numerator and denominator are removed");

  TransferFcn tfcn;
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