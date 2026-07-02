#include <gtest/gtest.h>

#include <cmath>

#include "common/numerics/ClampUtils.h"

namespace oneq {
namespace common {
namespace numerics {
namespace {

TEST(NumericClampUtilsTest, NormalizeAngle180HandlesVeryLargeInput) {
  const float normalized = NormalizeAngle180(1.0e30f);

  EXPECT_TRUE(std::isfinite(normalized));
  EXPECT_GE(normalized, -180.0f);
  EXPECT_LE(normalized, 180.0f);
}

TEST(NumericClampUtilsTest, NormalizeAngle180KeepsExpectedWrappedValues) {
  EXPECT_FLOAT_EQ(NormalizeAngle180(0.0f), 0.0f);
  EXPECT_FLOAT_EQ(NormalizeAngle180(181.0f), -179.0f);
  EXPECT_FLOAT_EQ(NormalizeAngle180(-181.0f), 179.0f);
  EXPECT_FLOAT_EQ(NormalizeAngle180(540.0f), -180.0f);
}

}  // namespace
}  // namespace numerics
}  // namespace common
}  // namespace oneq
