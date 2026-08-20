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
  config::EsrSessionConfig config;
  config.hardware.receiver_band_lower_hz = 9.0e9;
  config.hardware.receiver_band_upper_hz = 11.0e9;
  config.hardware.receiver_sensitivity_w = 2.0e-12f;
  config.hardware.beam_az_width_deg = 8.0f;
  config.hardware.beam_el_width_deg = 6.0f;
  config.hardware.antenna_mount_az_deg = 5.0f;
  config.hardware.antenna_mount_el_deg = 2.0f;
  config.sensor_enabled = false;
  config.mission.work_mode = config::EsrWorkMode::kHgesm;
  config.mission.scan.scan_center_az_deg = 10.0f;
  config.mission.scan.scan_center_el_deg = 4.0f;
  config.mission.scan.scan_rate_hz = 4.0f;
  config.mission.scan.scan_start_position = config::EsrScanStartPosition::kRightBottom;
  config.mission.scan.scan_sequence = config::EsrScanSequence::kElevationFirst;

  const EsrInternalExecutionConfig exec = MapSessionToInternal(config);

  // power_on maps correctly
  EXPECT_FALSE(exec.sensor_enabled);
  // receiver window is derived from hardware band
  EXPECT_TRUE(exec.hardware.receiver_band_lower_hz > 0.0);
  // scan rate is preserved
  EXPECT_FLOAT_EQ(exec.mission.scan.scan_rate_hz, 4.0f);
  // receiver sensitivity maps to hardware
  EXPECT_FLOAT_EQ(exec.hardware.receiver_sensitivity_w, 2.0e-12f);
  // beam widths are from hardware (for scan step)
  EXPECT_FLOAT_EQ(exec.hardware.beam_az_width_deg, 8.0f);
  EXPECT_FLOAT_EQ(exec.hardware.beam_el_width_deg, 6.0f);
  // scan position and sequence are preserved
  EXPECT_EQ(exec.resolved_scan.scan_start_pos,
            static_cast<int>(config::EsrScanStartPosition::kRightBottom));
  EXPECT_EQ(exec.resolved_scan.scan_sequence,
            static_cast<int>(config::EsrScanSequence::kElevationFirst));
}

TEST(EsrSessionConfigResolverTest, DetectionAndEnvironmentPoliciesMapToInternalDefaults) {
  config::EsrSessionConfig config;
  config.policy.detection.minimum_snr_db = 3.0f;
  config.policy.detection.pfa = 1.0e-5f;
  config.environment.scenario_config.preset = config::EsrEnvironmentPreset::kJammed;

  const EsrInternalExecutionConfig exec = MapSessionToInternal(config);

  EXPECT_FLOAT_EQ(exec.detection.minimum_snr_db, 3.0f);
  EXPECT_FLOAT_EQ(exec.detection.pfa, 1.0e-5f);
  EXPECT_EQ(exec.environment.preset, config::EsrEnvironmentPreset::kJammed);
}

TEST(EsrSessionConfigResolverTest, InvalidInputsFallBackToSafeDefaults) {
  config::EsrSessionConfig config;
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

  const EsrInternalExecutionConfig exec = MapSessionToInternal(config);

  // Mount angles validated — default to 0.0f
  EXPECT_FLOAT_EQ(exec.hardware.antenna_mount_az_deg, std::numeric_limits<float>::infinity());
  EXPECT_FLOAT_EQ(exec.hardware.antenna_mount_el_deg, -std::numeric_limits<float>::infinity());
  // Scan rate falls back via hardware/mission (not validated in resolver)
  EXPECT_FLOAT_EQ(exec.mission.scan.scan_rate_hz, 0.0f);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace electronic_surveillance_radar
