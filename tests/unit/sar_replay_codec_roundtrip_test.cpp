/**
 * @file sar_replay_codec_roundtrip_test.cpp
 * @brief 验证 SAR replay FlatBuffers codec 的 Encode/Decode 字段保真。
 */

#include <gtest/gtest.h>

#include "1q/sar/config/SarRuntimeConfigPatch.h"
#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"
#include "sar/session/SarReplayFlatbufferCodec.h"

namespace sar {
namespace session {
namespace tests {

TEST(SarReplayCodecRoundtripTest, CycleInputPreservesPlatformAndTargets) {
  SarCycleInput input;
  input.cycle_index = 7U;
  input.dt_sec = 0.25;
  input.platform.time_s = 12.5;
  input.platform.latitude_deg = 31.2;
  input.platform.longitude_deg = 121.4;
  input.platform.altitude_m = 3500.0;
  input.platform.velocity_north_mps = 120.0;
  input.platform.velocity_east_mps = 15.0;
  input.platform.velocity_down_mps = -1.0;
  input.platform.roll_deg = 1.5;
  input.platform.pitch_deg = -2.0;
  input.platform.yaw_deg = 88.0;
  SarPointTarget target;
  target.latitude_deg = 31.2001;
  target.longitude_deg = 121.4002;
  target.altitude_m = 15.0;
  target.radar_cross_section_dbsm = 12.0;
  input.point_targets.push_back(target);

  const std::string bytes = EncodeSarCycleInput(input);
  ASSERT_FALSE(bytes.empty());
  SarCycleInput decoded;
  ASSERT_TRUE(DecodeSarCycleInput(bytes, &decoded));

  EXPECT_EQ(decoded.cycle_index, input.cycle_index);
  EXPECT_DOUBLE_EQ(decoded.dt_sec, input.dt_sec);
  EXPECT_DOUBLE_EQ(decoded.platform.latitude_deg, input.platform.latitude_deg);
  EXPECT_DOUBLE_EQ(decoded.platform.velocity_east_mps, input.platform.velocity_east_mps);
  EXPECT_DOUBLE_EQ(decoded.platform.yaw_deg, input.platform.yaw_deg);
  ASSERT_EQ(decoded.point_targets.size(), 1U);
  EXPECT_DOUBLE_EQ(decoded.point_targets[0].longitude_deg, target.longitude_deg);
  EXPECT_DOUBLE_EQ(decoded.point_targets[0].radar_cross_section_dbsm,
                   target.radar_cross_section_dbsm);
}

TEST(SarReplayCodecRoundtripTest, CycleResultPreservesOutputAndDiagnostics) {
  SarCycleResult result;
  result.input_cycle_index = 9U;
  result.output_frame.cycle_index = 9U;
  result.output_frame.completed_stage = SarProcessingStage::kL1RdaImage;
  result.output_frame.range_sample_count = 64U;
  result.output_frame.azimuth_pulse_count = 9U;
  result.output_frame.center_slant_range_m = 29.9792458;
  result.output_frame.estimated_snr_db = 18.5;
  result.output_frame.has_raw_echo = true;
  result.output_frame.has_range_compressed_echo = true;
  result.output_frame.has_l1_image = true;
  SarDiagnosticIssue diagnostic;
  diagnostic.severity = SarDiagnosticSeverity::kInfo;
  diagnostic.code = "sar.rda_peak";
  diagnostic.message = "peak index";
  result.diagnostics.push_back(diagnostic);
  result.executed_this_cycle = true;

  const std::string bytes = EncodeSarCycleResult(result);
  ASSERT_FALSE(bytes.empty());
  SarCycleResult decoded;
  ASSERT_TRUE(DecodeSarCycleResult(bytes, &decoded));

  EXPECT_EQ(decoded.input_cycle_index, result.input_cycle_index);
  EXPECT_EQ(decoded.output_frame.completed_stage, SarProcessingStage::kL1RdaImage);
  EXPECT_EQ(decoded.output_frame.range_sample_count, 64U);
  EXPECT_TRUE(decoded.output_frame.has_l1_image);
  ASSERT_EQ(decoded.diagnostics.size(), 1U);
  EXPECT_EQ(decoded.diagnostics[0].severity, SarDiagnosticSeverity::kInfo);
  EXPECT_EQ(decoded.diagnostics[0].code, "sar.rda_peak");
  EXPECT_TRUE(decoded.executed_this_cycle);
  EXPECT_FALSE(decoded.has_error);
}

TEST(SarReplayCodecRoundtripTest, SessionConfigPreservesAllDomains) {
  config::SarSessionConfig config;
  config.hardware.carrier_frequency_hz = 1.0e9;
  config.hardware.bandwidth_hz = 25.0e6;
  config.hardware.pulse_width_s = 0.16e-6;
  config.hardware.pulse_repetition_frequency_hz = 20.0;
  config.hardware.sample_rate_hz = 100.0e6;
  config.hardware.peak_power_w = 500.0;
  config.hardware.antenna_length_m = 1.5;
  config.hardware.antenna_width_m = 0.4;
  config.hardware.antenna_gain_db = 28.0;
  config.hardware.receiver_noise_figure_db = 3.5;
  config.hardware.system_loss_db = 2.0;
  config.mission.scene_center_latitude_deg = 30.0;
  config.mission.scene_center_longitude_deg = 120.0;
  config.mission.scene_center_altitude_m = 5.0;
  config.mission.nominal_slant_range_m = 29.9792458;
  config.mission.synthetic_aperture_time_s = 0.4;
  config.mission.platform_speed_mps = 2.0;
  config.mission.range_sample_count = 64U;
  config.mission.azimuth_pulse_count = 9U;
  config.mission.desired_ground_range_resolution_m = 2.0;
  config.mission.desired_azimuth_resolution_m = 3.0;
  config.policy.enable_raw_echo_generation = true;
  config.policy.enable_range_compression = true;
  config.policy.enable_l1_rda_imaging = true;
  config.policy.enable_diagnostics = false;
  config.policy.retain_raw_phase_history = true;
  config.policy.max_allowed_squint_angle_deg = 3.0;
  config.policy.min_valid_snr_db = -5.0;
  config.environment.terrain_reference_altitude_m = 4.0;
  config.environment.atmospheric_loss_db_per_km = 0.1;
  config.environment.surface_backscatter_sigma0_db = -9.0;
  config.environment.use_flat_earth_geometry = false;
  config.environment.enable_atmospheric_attenuation = true;

  const std::string bytes = EncodeSarSessionConfig(config);
  ASSERT_FALSE(bytes.empty());
  config::SarSessionConfig decoded;
  ASSERT_TRUE(DecodeSarSessionConfig(bytes, &decoded));

  EXPECT_DOUBLE_EQ(decoded.hardware.carrier_frequency_hz, config.hardware.carrier_frequency_hz);
  EXPECT_DOUBLE_EQ(decoded.hardware.system_loss_db, config.hardware.system_loss_db);
  EXPECT_DOUBLE_EQ(decoded.mission.scene_center_longitude_deg,
                   config.mission.scene_center_longitude_deg);
  EXPECT_EQ(decoded.mission.azimuth_pulse_count, 9U);
  EXPECT_TRUE(decoded.policy.enable_l1_rda_imaging);
  EXPECT_TRUE(decoded.policy.retain_raw_phase_history);
  EXPECT_DOUBLE_EQ(decoded.policy.min_valid_snr_db, -5.0);
  EXPECT_FALSE(decoded.environment.use_flat_earth_geometry);
  EXPECT_TRUE(decoded.environment.enable_atmospheric_attenuation);
}

TEST(SarReplayCodecRoundtripTest, RuntimeConfigPatchPreservesHasBitsAndValues) {
  config::SarRuntimeConfigPatch patch;
  patch.has_enable_raw_echo_generation = true;
  patch.enable_raw_echo_generation = false;
  patch.has_enable_range_compression = true;
  patch.enable_range_compression = false;
  patch.has_enable_l1_rda_imaging = true;
  patch.enable_l1_rda_imaging = true;
  patch.has_retain_raw_phase_history = true;
  patch.retain_raw_phase_history = true;
  patch.has_min_valid_snr_db = true;
  patch.min_valid_snr_db = 6.5;

  const std::string bytes = EncodeSarRuntimeConfigPatch(patch);
  ASSERT_FALSE(bytes.empty());
  config::SarRuntimeConfigPatch decoded;
  ASSERT_TRUE(DecodeSarRuntimeConfigPatch(bytes, &decoded));

  EXPECT_TRUE(decoded.has_enable_raw_echo_generation);
  EXPECT_FALSE(decoded.enable_raw_echo_generation);
  EXPECT_TRUE(decoded.has_enable_range_compression);
  EXPECT_FALSE(decoded.enable_range_compression);
  EXPECT_TRUE(decoded.has_enable_l1_rda_imaging);
  EXPECT_TRUE(decoded.enable_l1_rda_imaging);
  EXPECT_TRUE(decoded.has_retain_raw_phase_history);
  EXPECT_TRUE(decoded.retain_raw_phase_history);
  EXPECT_TRUE(decoded.has_min_valid_snr_db);
  EXPECT_DOUBLE_EQ(decoded.min_valid_snr_db, 6.5);
}

TEST(SarReplayCodecRoundtripTest, RejectsEmptyPayload) {
  SarCycleInput input;
  EXPECT_FALSE(DecodeSarCycleInput("", &input));
  SarCycleResult result;
  EXPECT_FALSE(DecodeSarCycleResult("", &result));
  config::SarSessionConfig config;
  EXPECT_FALSE(DecodeSarSessionConfig("", &config));
}

}  // namespace tests
}  // namespace session
}  // namespace sar
