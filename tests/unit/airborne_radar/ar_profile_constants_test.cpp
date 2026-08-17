// @file ar_profile_constants_test.cpp
// @brief 验证 ArProfileConstants 常量字段值与旧 Builder 翻译输出一致（迁移锚点）。

#include <gtest/gtest.h>

#include "1q/airborne_radar/config/ArProfileConstants.h"
#include "1q/airborne_radar/config/ArSessionConfigValidation.h"

namespace airborne_radar {
namespace config {
namespace tests {

TEST(ArProfileConstantsTest, LongRangeHighPowerHardware) {
  const auto& c = profiles::kLongRangeHighPowerHardware;
  EXPECT_FLOAT_EQ(c.transmitter.peak_power_w, 5.0e6f);
  EXPECT_FLOAT_EQ(c.transmitter.maximum_peak_power_w, 6.0e6f);
  EXPECT_FLOAT_EQ(c.transmitter.frequency_hz, 9.3e9f);
  EXPECT_EQ(c.transmitter.frequency_plan_hz.size(), 1U);
  // plan 首项必须与 frequency_hz 严格相等（校验要求 plan 包含初始载频）。
  EXPECT_DOUBLE_EQ(c.transmitter.frequency_plan_hz[0],
                   static_cast<double>(c.transmitter.frequency_hz));
  EXPECT_FLOAT_EQ(c.transmitter.bandwidth_hz, 3.0e6f);
  EXPECT_FLOAT_EQ(c.transmitter.pulse_width_s, 18e-6f);
  EXPECT_FLOAT_EQ(c.transmitter.prf_hz, 220.0f);
  EXPECT_FLOAT_EQ(c.antenna.main_beam_gain_db, 38.0f);
  EXPECT_FLOAT_EQ(c.receiver.noise_figure_db, 3.0f);
}

TEST(ArProfileConstantsTest, LongRangeHighPowerHardwarePassesValidation) {
  // 回归锚点：frequency_plan_hz 首项必须与 frequency_hz 的浮点表示严格相等，
  // 否则 ValidateArSessionConfig 报 kFrequencyPlanInvalid。
  // 注意：kLongRangeHighPower 档位本身不满足发射操作包络（pulse_energy=90J > struct 默认
  // 20J 上限），与旧 Builder 档位行为一致（旧档位同样不设 maximum_pulse_energy_j）；
  // 显式放宽包络后配置应通过全部校验。
  config::ArSessionConfig config;
  config.hardware = profiles::kLongRangeHighPowerHardware;
  config.hardware.transmitter.maximum_pulse_energy_j = 100.0f;
  EXPECT_TRUE(ValidateArSessionConfig(config).empty());
}

TEST(ArProfileConstantsTest, LightweightLpiHardware) {
  const auto& c = profiles::kLightweightLpiHardware;
  EXPECT_FLOAT_EQ(c.transmitter.peak_power_w, 3.5e5f);
  EXPECT_FLOAT_EQ(c.transmitter.frequency_hz, 10.0e9f);
  EXPECT_EQ(c.transmitter.frequency_plan_hz.size(), 1U);
  EXPECT_DOUBLE_EQ(c.transmitter.frequency_plan_hz[0], 10.0e9);
  EXPECT_FLOAT_EQ(c.transmitter.bandwidth_hz, 8.0e6f);
  EXPECT_FLOAT_EQ(c.transmitter.pulse_width_s, 8e-6f);
  EXPECT_FLOAT_EQ(c.transmitter.prf_hz, 600.0f);
  EXPECT_FLOAT_EQ(c.antenna.main_beam_gain_db, 31.0f);
  EXPECT_FLOAT_EQ(c.antenna.nominal_az_beamwidth_deg, 5.0f);
  EXPECT_FLOAT_EQ(c.antenna.nominal_el_beamwidth_deg, 5.0f);
  EXPECT_FLOAT_EQ(c.receiver.noise_figure_db, 5.0f);
}

TEST(ArProfileConstantsTest, DetectionPriorityDetection) {
  const auto& c = profiles::kDetectionPriorityDetection;
  EXPECT_EQ(c.pulse_count, 16);
  EXPECT_FLOAT_EQ(c.pfa, 2e-6f);
  EXPECT_FLOAT_EQ(c.minimum_snr_db, -12.0f);
  EXPECT_FLOAT_EQ(c.minimum_detection_margin_db, -100.0f);
}

TEST(ArProfileConstantsTest, TrackStabilityPriorityDetection) {
  const auto& c = profiles::kTrackStabilityPriorityDetection;
  EXPECT_EQ(c.pulse_count, 8);
  EXPECT_FLOAT_EQ(c.pfa, 5e-7f);
  EXPECT_FLOAT_EQ(c.minimum_snr_db, -8.0f);
  EXPECT_FLOAT_EQ(c.minimum_detection_margin_db, -20.0f);
}

TEST(ArProfileConstantsTest, LowSidelobeAntennaPattern) {
  const auto& c = profiles::kLowSidelobeAntennaPattern;
  EXPECT_FLOAT_EQ(c.max_sidelobe_level_db, -30.0f);
  EXPECT_FLOAT_EQ(c.backlobe_level_db, -42.0f);
}

TEST(ArProfileConstantsTest, WideCoverageAntennaPattern) {
  const auto& c = profiles::kWideCoverageAntennaPattern;
  EXPECT_EQ(c.model_type, detection::AntennaPatternModelType::kParabolicMainLobe);
  EXPECT_FLOAT_EQ(c.max_sidelobe_level_db, -18.0f);
  EXPECT_FLOAT_EQ(c.max_scan_loss_db, 8.0f);
}

TEST(ArProfileConstantsTest, ConservativeRcsPhysics) {
  const auto& c = profiles::kConservativeRcsPhysics;
  EXPECT_TRUE(c.enable_physical_rcs);
  EXPECT_FLOAT_EQ(c.physics_mix_ratio, 0.25f);
}

TEST(ArProfileConstantsTest, EnhancedRcsPhysics) {
  const auto& c = profiles::kEnhancedRcsPhysics;
  EXPECT_TRUE(c.enable_physical_rcs);
  EXPECT_FLOAT_EQ(c.physics_mix_ratio, 0.60f);
  EXPECT_FLOAT_EQ(c.cylinder_weight, 0.65f);
}

TEST(ArProfileConstantsTest, FastAssociationTracking) {
  const auto& c = profiles::kFastAssociationTracking;
  EXPECT_FLOAT_EQ(c.kalman_measurement_noise_std, 6.0f);
  EXPECT_FLOAT_EQ(c.speed_decay_ratio_on_loss, 0.95f);
  EXPECT_FLOAT_EQ(c.rcs_decay_ratio_on_loss, 0.92f);
}

TEST(ArProfileConstantsTest, RobustAntiJammingTracking) {
  const auto& c = profiles::kRobustAntiJammingTracking;
  EXPECT_FLOAT_EQ(c.kalman_measurement_noise_std, 12.0f);
  EXPECT_FLOAT_EQ(c.speed_decay_ratio_on_loss, 0.95f);
  EXPECT_FLOAT_EQ(c.rcs_decay_ratio_on_loss, 0.92f);
}

TEST(ArProfileConstantsTest, RobustAntiJammingAssociation) {
  const auto& c = profiles::kRobustAntiJammingAssociation;
  // 字面量钉死实际值（sqrt(12) ≈ 3.4641016151），防止常量与测试同表达式互相掩盖漂移。
  EXPECT_FLOAT_EQ(c.distance_gate_sigma, 3.4641016f);
}

TEST(ArProfileConstantsTest, GenericAirborneXBandHardwareIsStructDefault) {
  // kGenericAirborneXBand 是 no-op 档位：struct 默认即该档位，逐字段锁定防漂移。
  const detection::DetectionConfig default_hardware{};
  EXPECT_FLOAT_EQ(default_hardware.transmitter.peak_power_w, 1e6f);
  EXPECT_FLOAT_EQ(default_hardware.transmitter.frequency_hz, 3e9f);
  EXPECT_FLOAT_EQ(default_hardware.transmitter.bandwidth_hz, 4.5e6f);
  EXPECT_EQ(default_hardware.transmitter.frequency_plan_hz.size(), 1U);
  EXPECT_DOUBLE_EQ(default_hardware.transmitter.frequency_plan_hz[0],
                   static_cast<double>(default_hardware.transmitter.frequency_hz));
  EXPECT_FLOAT_EQ(default_hardware.antenna.main_beam_gain_db, 35.0f);
  EXPECT_FLOAT_EQ(default_hardware.receiver.noise_figure_db, 4.0f);
  EXPECT_FLOAT_EQ(default_hardware.signal_processing.target_processing_gain_db, 0.0f);
  EXPECT_FLOAT_EQ(default_hardware.signal_processing.noise_processing_gain_db, 0.0f);
  EXPECT_FLOAT_EQ(default_hardware.signal_processing.clutter_suppression_gain_db, 0.0f);
  EXPECT_FLOAT_EQ(default_hardware.signal_processing.jamming_suppression_gain_db, 0.0f);
}

TEST(ArProfileConstantsTest, FastConfirmLifecycle) {
  const auto& c = profiles::kFastConfirmLifecycle;
  EXPECT_EQ(c.confirm_hits, 1U);
  EXPECT_EQ(c.max_miss_before_lost, 1U);
  EXPECT_EQ(c.max_lost_cycles, 3U);
}

TEST(ArProfileConstantsTest, HighPersistenceLifecycle) {
  const auto& c = profiles::kHighPersistenceLifecycle;
  EXPECT_EQ(c.confirm_hits, 3U);
  EXPECT_EQ(c.max_miss_before_lost, 3U);
  EXPECT_EQ(c.max_lost_cycles, 8U);
}

}  // namespace tests
}  // namespace config
}  // namespace airborne_radar
