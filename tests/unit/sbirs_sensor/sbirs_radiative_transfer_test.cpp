#include <gtest/gtest.h>

#include "sbirs_sensor/foundation/SbirsRadiometry.h"

namespace {

TEST(SbirsRadiativeTransferTest, TransmittanceScalesReceivedPower) {
  const double radiance = sbirs_sensor::foundation::ComputeBandRadiance(3.0, 5.0, 1800.0);
  const double clear_power = sbirs_sensor::foundation::ComputeReceivedPowerW(
      radiance, 100.0, 1000000.0, 0.5, 0.8, 1.0, 0.9);
  const double attenuated_power = sbirs_sensor::foundation::ComputeReceivedPowerW(
      radiance, 100.0, 1000000.0, 0.5, 0.8, 0.2, 0.9);

  EXPECT_GT(clear_power, attenuated_power);
  EXPECT_GT(attenuated_power, 0.0);
}

}  // namespace
