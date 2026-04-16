/**
 * @file esr_session_config_resolver_test.cpp
 * @brief ESR 会话配置解析器测试（高层语义输入 -> 内部参数）。
 */

#include <gtest/gtest.h>

#include <limits>

#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "electronic_surveillance_radar/session/EsrSessionConfigResolver.h"

namespace electronic_surveillance_radar {
namespace session {
namespace internal {
namespace {

TEST(EsrSessionConfigResolverTest, HardwareAndMissionMapToRuntimeAndScanConfig) {
  EsrSessionConfig config;
  config.hardware.receiver_band_lower_hz = 9.0e9;
  config.hardware.receiver_band_upper_hz = 11.0e9;
  config.hardware.receiver_sensitivity_w = 2.0e-12f;
  config.hardware.beam_az_width_deg = 8.0f;
  config.hardware.beam_el_width_deg = 6.0f;
  config.hardware.antenna_mount_az_deg = 5.0f;
  config.hardware.antenna_mount_el_deg = 2.0f;
  config.mission.power_on = false;
  config.mission.work_mode = EsrWorkMode::kHgesm;
  config.mission.scan.scan_center_az_deg = 10.0f;
  config.mission.scan.scan_center_el_deg = 4.0f;
  config.mission.scan.scan_rate_hz = 4.0f;
  config.mission.scan.scan_start_position = EsrScanStartPosition::kRightBottom;
  config.mission.scan.scan_sequence = EsrScanSequence::kElevationFirst;

  const ResolvedEsrSessionConfig resolved = ResolveEsrSessionConfig(config);

  EXPECT_FALSE(resolved.runtime_config.sensor_enabled);
  EXPECT_TRUE(resolved.runtime_config.use_fixed_receiver_window);
  EXPECT_DOUBLE_EQ(resolved.runtime_config.receiver_lower_hz, 9.0e9);
  EXPECT_DOUBLE_EQ(resolved.runtime_config.receiver_upper_hz, 11.0e9);
  EXPECT_FLOAT_EQ(resolved.runtime_config.scan_rate_hz, 4.0f);
  EXPECT_FLOAT_EQ(resolved.pipeline_config.detection.receiver_noise_floor_w, 2.0e-12f);
  EXPECT_FLOAT_EQ(resolved.pipeline_config.scan.az_step_deg, 8.0f);
  EXPECT_FLOAT_EQ(resolved.pipeline_config.scan.el_step_deg, 6.0f);
  EXPECT_EQ(resolved.pipeline_config.scan.scan_start_pos,
            static_cast<int>(EsrScanStartPosition::kRightBottom));
  EXPECT_EQ(resolved.pipeline_config.scan.scan_sequence,
            static_cast<int>(EsrScanSequence::kElevationFirst));
}

TEST(EsrSessionConfigResolverTest, DetectionAndEnvironmentPoliciesMapToInternalDefaults) {
  EsrSessionConfig config;
  config.detection.profile = config::EsrDetectionProfile::kSensitive;
  config.environment.preset = config::EsrEnvironmentPreset::kJammed;

  const ResolvedEsrSessionConfig resolved = ResolveEsrSessionConfig(config);

  EXPECT_FLOAT_EQ(resolved.pipeline_config.detection.min_detect_snr_db, 3.0f);
  EXPECT_FLOAT_EQ(resolved.environment_model_config.default_clutter_noise_w, 1.0e-11f);
  EXPECT_FLOAT_EQ(resolved.environment_model_config.jamming_detection_threshold_w, 5.0e-9f);
}

TEST(EsrSessionConfigResolverTest, InvalidInputsFallBackToSafeDefaults) {
  EsrSessionConfig config;
  config.hardware.receiver_band_lower_hz = 12.0e9;
  config.hardware.receiver_band_upper_hz = 9.0e9;
  config.hardware.receiver_sensitivity_w = -1.0f;
  config.hardware.integrated_receive_loss_db = std::numeric_limits<float>::quiet_NaN();
  config.hardware.antenna_mount_az_deg = std::numeric_limits<float>::infinity();
  config.hardware.antenna_mount_el_deg = -std::numeric_limits<float>::infinity();
  config.mission.scan.scan_rate_hz = 0.0f;
  config.mission.scan.use_explicit_scan_bounds = true;
  config.mission.scan.scan_start_az_deg = std::numeric_limits<float>::quiet_NaN();
  config.mission.scan.scan_end_az_deg = std::numeric_limits<float>::quiet_NaN();
  config.mission.scan.scan_start_el_deg = std::numeric_limits<float>::quiet_NaN();
  config.mission.scan.scan_end_el_deg = std::numeric_limits<float>::quiet_NaN();

  const ResolvedEsrSessionConfig resolved = ResolveEsrSessionConfig(config);

  EXPECT_FALSE(resolved.runtime_config.use_fixed_receiver_window);
  EXPECT_FLOAT_EQ(resolved.runtime_config.integrated_receive_loss_db, 0.0f);
  EXPECT_FLOAT_EQ(resolved.runtime_config.antenna_mount_az_deg, 0.0f);
  EXPECT_FLOAT_EQ(resolved.runtime_config.antenna_mount_el_deg, 0.0f);
  EXPECT_FLOAT_EQ(resolved.runtime_config.scan_rate_hz, 1.0f);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace electronic_surveillance_radar
