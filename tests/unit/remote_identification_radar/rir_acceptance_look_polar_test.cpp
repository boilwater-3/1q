// RIR 航迹 ENU → 验收斜距/方位/俯仰（与 ComputeLookAngles 同口径）。

#include "remote_identification_radar/runtime/RirAcceptanceRecords.h"

#include <cmath>
#include <limits>

#include "gtest/gtest.h"

namespace remote_identification_radar {
namespace runtime {
namespace {

TEST(RirAcceptanceLookPolarTest, EastTwentyDegElevationMatchesLongRangeGeometry) {
  float range_m = 0.0f;
  float az_deg = 0.0f;
  float el_deg = 0.0f;
  ASSERT_TRUE(TryLookPolarFromEnuM(3194955.0f, 0.0f, 1162868.0f, &range_m, &az_deg, &el_deg));
  EXPECT_NEAR(range_m, 3400000.0f, 5.0f);
  EXPECT_NEAR(az_deg, 0.0f, 1.0e-3f);
  EXPECT_NEAR(el_deg, 20.0f, 0.02f);
}

TEST(RirAcceptanceLookPolarTest, NorthIsNinetyDegAzimuthFromEast) {
  float range_m = 0.0f;
  float az_deg = 0.0f;
  float el_deg = 0.0f;
  ASSERT_TRUE(TryLookPolarFromEnuM(0.0f, 1000.0f, 0.0f, &range_m, &az_deg, &el_deg));
  EXPECT_NEAR(range_m, 1000.0f, 1.0e-3f);
  EXPECT_NEAR(az_deg, 90.0f, 1.0e-3f);
  EXPECT_NEAR(el_deg, 0.0f, 1.0e-3f);
}

TEST(RirAcceptanceLookPolarTest, NearOriginAndNullOutputsAreInvalid) {
  float range_m = 1.0f;
  float az_deg = 1.0f;
  float el_deg = 1.0f;
  EXPECT_FALSE(TryLookPolarFromEnuM(0.0f, 0.0f, 0.0f, &range_m, &az_deg, &el_deg));
  EXPECT_FALSE(TryLookPolarFromEnuM(0.05f, 0.0f, 0.0f, &range_m, &az_deg, &el_deg));
  EXPECT_FALSE(TryLookPolarFromEnuM(1000.0f, 0.0f, 0.0f, nullptr, &az_deg, &el_deg));
  const float inf = std::numeric_limits<float>::infinity();
  EXPECT_FALSE(TryLookPolarFromEnuM(inf, 0.0f, 0.0f, &range_m, &az_deg, &el_deg));
}

}  // namespace
}  // namespace runtime
}  // namespace remote_identification_radar
