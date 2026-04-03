/**
 * @file esr_session_config_resolver_test.cpp
 * @brief 验证 ESR 会话分层参数解析器的覆盖优先级与回退行为。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "1q/electronic_surveillance_radar/core/session/EsrSession.h"
#include "electronic_surveillance_radar/core/session/EsrSessionConfigResolver.h"

namespace electronic_surveillance_radar {
namespace core {
namespace session {
namespace internal {
namespace {

TEST(EsrSessionConfigResolverTest, LayeredConfigDisabledKeepsLegacyConfig) {
  EsrSessionConfig config;
  config.enable_layered_config = false;
  config.pipeline_config.detection.receiver_noise_floor_w = 7.0e-13f;
  config.pipeline_config.scan.scan_start_az_deg = -40.0f;
  config.pipeline_config.scan.scan_end_az_deg = 30.0f;
  config.pipeline_config.statistical_detection.pulse_count = 9U;
  config.pipeline_config.statistical_detection.threshold_scale = 1.2f;
  config.layered_config.hardware.receiver_sensitivity_w = 5.0e-12f;
  config.layered_config.mission.work_mode = EsrWorkMode::kRwr;
  config.layered_config.mission.power_on = false;

  const ResolvedEsrSessionConfig resolved = ResolveEsrSessionConfig(config);

  EXPECT_FLOAT_EQ(resolved.pipeline_config.detection.receiver_noise_floor_w, 7.0e-13f);
  EXPECT_FLOAT_EQ(resolved.pipeline_config.scan.scan_start_az_deg, -40.0f);
  EXPECT_FLOAT_EQ(resolved.pipeline_config.scan.scan_end_az_deg, 30.0f);
  EXPECT_EQ(resolved.pipeline_config.statistical_detection.pulse_count, 9U);
  EXPECT_FLOAT_EQ(resolved.pipeline_config.statistical_detection.threshold_scale, 1.2f);
  EXPECT_TRUE(resolved.runtime_config.sensor_enabled);
  EXPECT_FALSE(resolved.runtime_config.use_fixed_receiver_window);
}

TEST(EsrSessionConfigResolverTest, LayeredConfigEnabledOverridesCoreFields) {
  EsrSessionConfig config;
  config.enable_layered_config = true;
  config.pipeline_config.statistical_detection.pulse_count = 8U;
  config.pipeline_config.statistical_detection.threshold_scale = 1.0f;
  config.pipeline_config.scan.scan_start_az_deg = -60.0f;
  config.pipeline_config.scan.scan_end_az_deg = 60.0f;
  config.pipeline_config.scan.scan_start_el_deg = -10.0f;
  config.pipeline_config.scan.scan_end_el_deg = 10.0f;

  config.layered_config.hardware.receiver_band_lower_hz = 9.0e9;
  config.layered_config.hardware.receiver_band_upper_hz = 11.0e9;
  config.layered_config.hardware.receiver_sensitivity_w = 2.0e-12f;
  config.layered_config.hardware.integrated_receive_loss_db = 3.0f;
  config.layered_config.hardware.beam_az_width_deg = 8.0f;
  config.layered_config.hardware.beam_el_width_deg = 6.0f;
  config.layered_config.hardware.az_scan_range_deg = 40.0f;
  config.layered_config.hardware.el_scan_range_deg = 20.0f;
  config.layered_config.hardware.antenna_mount_az_deg = 5.0f;
  config.layered_config.hardware.antenna_mount_el_deg = 2.0f;

  config.layered_config.mission.power_on = false;
  config.layered_config.mission.work_mode = EsrWorkMode::kHgesm;
  config.layered_config.mission.scan_center_az_deg = 10.0f;
  config.layered_config.mission.scan_center_el_deg = 4.0f;
  config.layered_config.mission.scan_rate_hz = 4.0f;
  config.layered_config.mission.scan_start_position = EsrScanStartPosition::kRightBottom;
  config.layered_config.mission.scan_sequence = EsrScanSequence::kElevationFirst;
  config.layered_config.mission.use_explicit_scan_bounds = false;

  const ResolvedEsrSessionConfig resolved = ResolveEsrSessionConfig(config);

  EXPECT_FALSE(resolved.runtime_config.sensor_enabled);
  EXPECT_TRUE(resolved.runtime_config.use_fixed_receiver_window);
  EXPECT_DOUBLE_EQ(resolved.runtime_config.receiver_lower_hz, 9.0e9);
  EXPECT_DOUBLE_EQ(resolved.runtime_config.receiver_upper_hz, 11.0e9);
  EXPECT_FLOAT_EQ(resolved.runtime_config.integrated_receive_loss_db, 3.0f);
  EXPECT_FLOAT_EQ(resolved.runtime_config.antenna_mount_az_deg, 5.0f);
  EXPECT_FLOAT_EQ(resolved.runtime_config.antenna_mount_el_deg, 2.0f);
  EXPECT_FLOAT_EQ(resolved.runtime_config.scan_rate_hz, 4.0f);

  EXPECT_FLOAT_EQ(resolved.pipeline_config.detection.receiver_noise_floor_w, 2.0e-12f);
  EXPECT_FLOAT_EQ(resolved.pipeline_config.scan.az_step_deg, 8.0f);
  EXPECT_FLOAT_EQ(resolved.pipeline_config.scan.el_step_deg, 6.0f);
  EXPECT_EQ(resolved.pipeline_config.scan.scan_start_pos,
            static_cast<int>(EsrScanStartPosition::kRightBottom));
  EXPECT_EQ(resolved.pipeline_config.scan.scan_sequence,
            static_cast<int>(EsrScanSequence::kElevationFirst));
  EXPECT_FLOAT_EQ(resolved.pipeline_config.scan.scan_start_az_deg, -15.0f);
  EXPECT_FLOAT_EQ(resolved.pipeline_config.scan.scan_end_az_deg, 25.0f);
  EXPECT_FLOAT_EQ(resolved.pipeline_config.scan.scan_start_el_deg, -8.0f);
  EXPECT_FLOAT_EQ(resolved.pipeline_config.scan.scan_end_el_deg, 12.0f);
  EXPECT_EQ(resolved.pipeline_config.statistical_detection.pulse_count, 32U);
  EXPECT_FLOAT_EQ(resolved.pipeline_config.statistical_detection.threshold_scale, 0.85f);
}

TEST(EsrSessionConfigResolverTest, InvalidLayeredValuesFallbackToLegacyOrSafeDefaults) {
  EsrSessionConfig config;
  config.enable_layered_config = true;
  config.pipeline_config.detection.receiver_noise_floor_w = 8.0e-13f;
  config.pipeline_config.scan.scan_start_az_deg = -30.0f;
  config.pipeline_config.scan.scan_end_az_deg = 50.0f;
  config.pipeline_config.scan.scan_start_el_deg = -12.0f;
  config.pipeline_config.scan.scan_end_el_deg = 16.0f;
  config.pipeline_config.statistical_detection.pulse_count = 10U;
  config.pipeline_config.statistical_detection.threshold_scale = 1.0f;

  config.layered_config.hardware.receiver_band_lower_hz = 12.0e9;
  config.layered_config.hardware.receiver_band_upper_hz = 9.0e9;
  config.layered_config.hardware.receiver_sensitivity_w = -1.0f;
  config.layered_config.hardware.integrated_receive_loss_db =
      std::numeric_limits<float>::quiet_NaN();
  config.layered_config.hardware.antenna_mount_az_deg = std::numeric_limits<float>::infinity();
  config.layered_config.hardware.antenna_mount_el_deg = -std::numeric_limits<float>::infinity();

  config.layered_config.mission.work_mode = EsrWorkMode::kRwr;
  config.layered_config.mission.scan_rate_hz = 0.0f;
  config.layered_config.mission.use_explicit_scan_bounds = true;
  config.layered_config.mission.scan_start_az_deg = std::numeric_limits<float>::quiet_NaN();
  config.layered_config.mission.scan_end_az_deg = std::numeric_limits<float>::quiet_NaN();
  config.layered_config.mission.scan_start_el_deg = std::numeric_limits<float>::quiet_NaN();
  config.layered_config.mission.scan_end_el_deg = std::numeric_limits<float>::quiet_NaN();
  config.layered_config.mission.scan_center_az_deg = std::numeric_limits<float>::quiet_NaN();
  config.layered_config.mission.scan_center_el_deg = std::numeric_limits<float>::quiet_NaN();

  const ResolvedEsrSessionConfig resolved = ResolveEsrSessionConfig(config);

  EXPECT_FALSE(resolved.runtime_config.use_fixed_receiver_window);
  EXPECT_FLOAT_EQ(resolved.pipeline_config.detection.receiver_noise_floor_w, 8.0e-13f);
  EXPECT_FLOAT_EQ(resolved.runtime_config.integrated_receive_loss_db, 0.0f);
  EXPECT_FLOAT_EQ(resolved.runtime_config.antenna_mount_az_deg, 0.0f);
  EXPECT_FLOAT_EQ(resolved.runtime_config.antenna_mount_el_deg, 0.0f);
  EXPECT_FLOAT_EQ(resolved.runtime_config.scan_rate_hz, 1.0f);
  EXPECT_FLOAT_EQ(resolved.pipeline_config.scan.scan_start_az_deg, -30.0f);
  EXPECT_FLOAT_EQ(resolved.pipeline_config.scan.scan_end_az_deg, 50.0f);
  EXPECT_FLOAT_EQ(resolved.pipeline_config.scan.scan_start_el_deg, -12.0f);
  EXPECT_FLOAT_EQ(resolved.pipeline_config.scan.scan_end_el_deg, 16.0f);
  EXPECT_EQ(resolved.pipeline_config.statistical_detection.pulse_count, 5U);
  EXPECT_FLOAT_EQ(resolved.pipeline_config.statistical_detection.threshold_scale, 1.25f);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace core
}  // namespace electronic_surveillance_radar
