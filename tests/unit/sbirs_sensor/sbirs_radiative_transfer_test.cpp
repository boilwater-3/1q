#include <gtest/gtest.h>

#include <cmath>

#include "sbirs_sensor/foundation/SbirsRadiometry.h"

namespace {

TEST(SbirsRadiativeTransferTest, TransmittanceScalesReceivedPower) {
  const double clear_power = sbirs_sensor::foundation::ComputeReceivedPowerW(
      1.0e4, 1000000.0, 0.5, 0.8, 1.0, 0.9);
  const double attenuated_power = sbirs_sensor::foundation::ComputeReceivedPowerW(
      1.0e4, 1000000.0, 0.5, 0.8, 0.2, 0.9);

  EXPECT_GT(clear_power, attenuated_power);
  EXPECT_GT(attenuated_power, 0.0);
}

// d_max 自洽性：SNR 链路在 d_max 处恰好达到检测门限（与实现公式互相校验）。
TEST(SbirsRadiativeTransferTest, MaxDetectionRangeMeetsThresholdExactly) {
  const double d_max = sbirs_sensor::foundation::ComputeMaxDetectionRangeM(
      1.0e4, 0.5, 0.8, 0.6, 0.9, 0.02, 1.0e-12, 4.0);
  EXPECT_GT(d_max, 0.0);
  const double received_power = sbirs_sensor::foundation::ComputeReceivedPowerW(
      1.0e4, d_max, 0.5, 0.8, 0.6, 0.9);
  const double snr_at_d_max = received_power * 0.02 / 1.0e-12;
  EXPECT_NEAR(snr_at_d_max, 4.0, 1.0e-6 * 4.0);
}

// 非正输入保护：签名/透过率/噪声/门限/积分为零时 d_max=0（无定义能力）。
TEST(SbirsRadiativeTransferTest, MaxDetectionRangeZeroForNonPositiveInputs) {
  EXPECT_DOUBLE_EQ(
      sbirs_sensor::foundation::ComputeMaxDetectionRangeM(0.0, 0.5, 0.8, 0.6, 0.9, 0.02, 1e-12, 4.0),
      0.0);
  EXPECT_DOUBLE_EQ(
      sbirs_sensor::foundation::ComputeMaxDetectionRangeM(1e4, 0.5, 0.8, 0.0, 0.9, 0.02, 1e-12, 4.0),
      0.0);
  EXPECT_DOUBLE_EQ(
      sbirs_sensor::foundation::ComputeMaxDetectionRangeM(1e4, 0.5, 0.8, 0.6, 0.9, 0.02, 0.0, 4.0),
      0.0);
  EXPECT_DOUBLE_EQ(
      sbirs_sensor::foundation::ComputeMaxDetectionRangeM(1e4, 0.5, 0.8, 0.6, 0.9, 0.02, 1e-12, 0.0),
      0.0);
  EXPECT_DOUBLE_EQ(
      sbirs_sensor::foundation::ComputeMaxDetectionRangeM(1e4, 0.5, 0.8, 0.6, 0.9, 0.0, 1e-12, 4.0),
      0.0);
}

// d_max ∝ SNR_th^(-1/2)（同一链路仅门限不同）；透过率下降 → d_max 变小（气象效果）。
TEST(SbirsRadiativeTransferTest, MaxDetectionRangeScalesWithThresholdAndTransmittance) {
  const double d_th1 = sbirs_sensor::foundation::ComputeMaxDetectionRangeM(
      1.0e4, 0.5, 0.8, 0.6, 0.9, 0.02, 1.0e-12, 1.0);
  const double d_th4 = sbirs_sensor::foundation::ComputeMaxDetectionRangeM(
      1.0e4, 0.5, 0.8, 0.6, 0.9, 0.02, 1.0e-12, 4.0);
  EXPECT_NEAR(d_th4 / d_th1, 0.5, 1.0e-12);

  const double d_clear = sbirs_sensor::foundation::ComputeMaxDetectionRangeM(
      1.0e4, 0.5, 0.8, 1.0, 0.9, 0.02, 1.0e-12, 4.0);
  const double d_fog = sbirs_sensor::foundation::ComputeMaxDetectionRangeM(
      1.0e4, 0.5, 0.8, 0.5, 0.9, 0.02, 1.0e-12, 4.0);
  EXPECT_LT(d_fog, d_clear);
  EXPECT_NEAR(d_fog / d_clear, 0.5 * std::sqrt(2.0), 1.0e-12);
}

}  // namespace
