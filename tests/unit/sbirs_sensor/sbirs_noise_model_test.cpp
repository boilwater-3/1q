// 验证 algorithms.md Foundation 物理链路条目：sbirs_noise_model_test
// 覆盖光子/热/读出噪声三项 RMS 合成与默认 NEP 回退。
#include <gtest/gtest.h>

#include "sbirs_sensor/foundation/SbirsNoiseModel.h"

namespace {

TEST(SbirsNoiseModelTest, DefaultsDecomposeToZeroAndFallBackToNep) {
  sbirs_sensor::config::SbirsHardwareConfig hardware;
  hardware.noise_equivalent_power_w = 1.0e-12f;
  // 显式关闭三项分解参数（默认 detector_temperature_k 非零会产生热噪声）。
  hardware.background_radiance_w_sr_m2 = 0.0f;
  hardware.detector_temperature_k = 0.0f;
  hardware.readout_noise_rms_w = 0.0f;
  const sbirs_sensor::foundation::SbirsNoiseStatistics stats =
      sbirs_sensor::foundation::ComputeBackgroundNoiseStatistics(hardware);
  // 三项分解参数全 0 → total 为 0。
  EXPECT_DOUBLE_EQ(stats.photon_noise_w, 0.0);
  EXPECT_DOUBLE_EQ(stats.total_noise_w, 0.0);
  // 回退到 NEP 标量（float→double 提升有精度差，用相对容差）。
  EXPECT_NEAR(sbirs_sensor::foundation::ResolveEffectiveNoiseW(hardware, stats), 1.0e-12, 1.0e-18);
}

TEST(SbirsNoiseModelTest, BackgroundRadianceProducesPhotonNoise) {
  sbirs_sensor::config::SbirsHardwareConfig hardware;
  hardware.background_radiance_w_sr_m2 = 1.0e-6f;
  hardware.optical_aperture_m = 0.5f;
  hardware.integration_time_sec = 0.02f;
  hardware.detector_temperature_k = 0.0f;  // 关闭热噪声以隔离光子项
  const sbirs_sensor::foundation::SbirsNoiseStatistics stats =
      sbirs_sensor::foundation::ComputeBackgroundNoiseStatistics(hardware);
  EXPECT_GT(stats.photon_noise_w, 0.0);
  // 总噪声应不小于任一单项。
  EXPECT_GE(stats.total_noise_w, stats.photon_noise_w);
}

TEST(SbirsNoiseModelTest, HigherDetectorTemperatureRaisesThermalNoise) {
  sbirs_sensor::config::SbirsHardwareConfig cold;
  cold.detector_temperature_k = 40.0f;
  cold.integration_time_sec = 0.02f;
  sbirs_sensor::config::SbirsHardwareConfig hot;
  hot.detector_temperature_k = 300.0f;
  hot.integration_time_sec = 0.02f;
  const sbirs_sensor::foundation::SbirsNoiseStatistics cold_stats =
      sbirs_sensor::foundation::ComputeBackgroundNoiseStatistics(cold);
  const sbirs_sensor::foundation::SbirsNoiseStatistics hot_stats =
      sbirs_sensor::foundation::ComputeBackgroundNoiseStatistics(hot);
  EXPECT_GT(hot_stats.thermal_noise_w, cold_stats.thermal_noise_w);
}

TEST(SbirsNoiseModelTest, RmsCompositionDominatedByLargestComponent) {
  // 读出噪声远大于其他项时，总噪声趋近读出噪声。关闭热/背景以隔离读出项。
  sbirs_sensor::config::SbirsHardwareConfig hardware;
  hardware.readout_noise_rms_w = 1.0e-9f;
  hardware.background_radiance_w_sr_m2 = 0.0f;
  hardware.detector_temperature_k = 0.0f;
  const sbirs_sensor::foundation::SbirsNoiseStatistics stats =
      sbirs_sensor::foundation::ComputeBackgroundNoiseStatistics(hardware);
  // 只剩读出项时 total == readout。
  EXPECT_NEAR(stats.total_noise_w, stats.readout_noise_w, 1.0e-18);
}

}  // namespace
