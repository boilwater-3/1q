/**
 * @file sar_replay_codec_roundtrip_test.cpp
 * @brief 验证 SAR replay FlatBuffers codec 的 Encode/Decode 字段保真。
 */

#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include "1q/sar/config/SarRuntimeConfigPatch.h"
#include "1q/sar/config/SarRuntimeConfigBuilder.h"
#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"
#include "flatbuffers/flatbuffers.h"
#include "sar/session/SarReplayFlatbufferCodec.h"
#include "sar/session/generated/sar_replay_generated.h"

namespace sar {
namespace session {
namespace tests {

namespace {

// 绕过 EncodeSarCycleResult 的合法值限制，直接构造携带任意 abort_reason 的帧。
// 注：decode 在 abort_reason 校验前要求 focused_image 及其向量非空，须构造合法空聚焦图像，
//     否则帧在更早的校验处被拒，abort_reason 守卫不会被真正测到。
std::string EncodeCycleResultWithRawAbortReason(std::int32_t abort_reason) {
  flatbuffers::FlatBufferBuilder builder(128U);
  const auto real = builder.CreateVector(std::vector<double>{});
  const auto imaginary = builder.CreateVector(std::vector<double>{});
  const auto focused_image =
      replay::CreateSarFocusedImage(builder, 0, 0, 0, real, imaginary, /*is_placeholder=*/true);
  builder.Finish(replay::CreateSarCycleResult(
      builder, 1U, 0, focused_image, 0, 0, abort_reason, 0));
  return std::string(reinterpret_cast<const char*>(builder.GetBufferPointer()),
                     builder.GetSize());
}

}  // namespace

TEST(SarReplayCodecRoundtripTest, CycleInputPreservesPlatformAndTargets) {
  SarCycleInput input;
  input.cycle_index = 7U;
  input.dt_sec = 0.25f;
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
  target.target_id = 501U;
  target.target_name = "sar-point-alpha";
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
  EXPECT_FLOAT_EQ(decoded.dt_sec, input.dt_sec);
  EXPECT_DOUBLE_EQ(decoded.platform.latitude_deg, input.platform.latitude_deg);
  EXPECT_DOUBLE_EQ(decoded.platform.velocity_east_mps, input.platform.velocity_east_mps);
  EXPECT_DOUBLE_EQ(decoded.platform.yaw_deg, input.platform.yaw_deg);
  ASSERT_EQ(decoded.point_targets.size(), 1U);
  EXPECT_EQ(decoded.point_targets[0].target_id, 501U);
  EXPECT_EQ(decoded.point_targets[0].target_name, "sar-point-alpha");
  EXPECT_DOUBLE_EQ(decoded.point_targets[0].longitude_deg, target.longitude_deg);
  EXPECT_DOUBLE_EQ(decoded.point_targets[0].radar_cross_section_dbsm,
                   target.radar_cross_section_dbsm);
}

TEST(SarReplayCodecRoundtripTest, CycleInputPreservesExternalRawIqAndDualTrajectories) {
  SarCycleInput input;
  input.cycle_index = 8U;
  input.raw_iq.samples_per_pulse = 3U;
  input.raw_iq.i_values = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  input.raw_iq.q_values = {-1.0, -2.0, -3.0, -4.0, -5.0, -6.0};
  for (std::uint64_t pulse_id = 10U; pulse_id < 12U; ++pulse_id) {
    SarRawIqFrame::PulseState actual;
    actual.pulse_id = pulse_id;
    actual.time_s = 0.25 * static_cast<double>(pulse_id - 10U);
    actual.position_x_m = static_cast<double>(pulse_id);
    actual.position_y_m = -2.0 * actual.position_x_m;
    actual.position_z_m = 1000.0;
    actual.velocity_x_mps = 120.0;
    actual.velocity_y_mps = 3.0;
    actual.velocity_z_mps = -0.5;
    input.raw_iq.pulse_states.push_back(actual);
    SarRawIqFrame::PulseState ideal = actual;
    ideal.position_y_m += 0.75;
    input.raw_iq.ideal_pulse_states.push_back(ideal);
  }

  const std::string bytes = EncodeSarCycleInput(input);
  ASSERT_FALSE(bytes.empty());
  SarCycleInput decoded;
  ASSERT_TRUE(DecodeSarCycleInput(bytes, &decoded));

  EXPECT_EQ(decoded.raw_iq.samples_per_pulse, input.raw_iq.samples_per_pulse);
  EXPECT_EQ(decoded.raw_iq.i_values, input.raw_iq.i_values);
  EXPECT_EQ(decoded.raw_iq.q_values, input.raw_iq.q_values);
  ASSERT_EQ(decoded.raw_iq.pulse_states.size(), 2U);
  ASSERT_EQ(decoded.raw_iq.ideal_pulse_states.size(), 2U);
  EXPECT_EQ(decoded.raw_iq.pulse_states[1].pulse_id, 11U);
  EXPECT_DOUBLE_EQ(decoded.raw_iq.pulse_states[1].position_y_m, -22.0);
  EXPECT_DOUBLE_EQ(decoded.raw_iq.ideal_pulse_states[1].position_y_m, -21.25);
  EXPECT_DOUBLE_EQ(decoded.raw_iq.pulse_states[0].velocity_z_mps, -0.5);
}

TEST(SarReplayCodecRoundtripTest, CycleInputDecodeFailureDoesNotModifyOutput) {
  SarCycleInput output;
  output.cycle_index = 77U;
  output.raw_iq.i_values = {123.0};

  EXPECT_FALSE(DecodeSarCycleInput("not-a-flatbuffer", &output));
  EXPECT_EQ(output.cycle_index, 77U);
  EXPECT_EQ(output.raw_iq.i_values, std::vector<double>({123.0}));
}

TEST(SarReplayCodecRoundtripTest, CycleResultPreservesOutputAndDiagnostics) {
  SarCycleResult result;
  result.input_cycle_index = 9U;
  result.output_frame.cycle_index = 9U;
  result.output_frame.completed_stage = SarProcessingStage::kL3BpImage;
  result.output_frame.range_sample_count = 64U;
  result.output_frame.azimuth_pulse_count = 9U;
  result.output_frame.center_slant_range_m = 29.9792458;
  result.output_frame.estimated_snr_db = 18.5;
  result.output_frame.phase_reference_mode = SarPhaseReferenceMode::kCenterBroadside;
  result.output_frame.image_quality_mainlobe_method = SarMainlobeEstimationMethod::k20dB;
  result.output_frame.range_width_3db_bins = 3.0;
  result.output_frame.azimuth_width_3db_bins = 5.0;
  result.output_frame.range_resolution_3db_m = 4.5;
  result.output_frame.azimuth_resolution_3db_m = 10.0;
  result.output_frame.image_entropy_nats = 2.25;
  result.output_frame.image_contrast = 1.5;
  result.output_frame.has_raw_echo = true;
  result.output_frame.has_range_compressed_echo = true;
  result.output_frame.has_l3_bp_image = true;
  result.output_frame.has_image_quality_metrics = true;
  result.output_frame.image_resolution_m_valid = true;
  result.output_frame.phase_reference_applied = true;
  SarIssue issue;
  issue.severity = SarIssueSeverity::kInfo;
  issue.phase = SarIssuePhase::kExecution;
  issue.code = "sar.rda_peak";
  issue.message = "peak index";
  // kPlatform 定位（校验实际使用形态）：验证非 kGlobal kind 经 codec 不丢失。
  issue.location.kind = oneq::foundation::ValidationLocationKind::kPlatform;
  issue.location.entity_index = static_cast<std::size_t>(-1);
  issue.field = "platform";
  result.issues.push_back(issue);
  result.raw_phase_history.source = SarRawPhaseHistorySource::kExternalRawIq;
  result.raw_phase_history.pulse_count = 2U;
  result.raw_phase_history.samples_per_pulse = 2U;
  result.raw_phase_history.i_values = {1.0, 2.0, 3.0, 4.0};
  result.raw_phase_history.q_values = {-1.0, -2.0, -3.0, -4.0};
  result.focused_image.source = SarFocusedImageSource::kL3Bp;
  result.focused_image.row_count = 2U;
  result.focused_image.column_count = 2U;
  result.focused_image.real_values = {10.0, 20.0, 30.0, 40.0};
  result.focused_image.imaginary_values = {-10.0, -20.0, -30.0, -40.0};
  result.status = SarCycleStatus::kCompleted;

  const std::string bytes = EncodeSarCycleResult(result);
  ASSERT_FALSE(bytes.empty());
  SarCycleResult decoded;
  ASSERT_TRUE(DecodeSarCycleResult(bytes, &decoded));

  EXPECT_EQ(decoded.input_cycle_index, result.input_cycle_index);
  EXPECT_EQ(decoded.output_frame.completed_stage, SarProcessingStage::kL3BpImage);
  EXPECT_EQ(decoded.output_frame.range_sample_count, 64U);
  EXPECT_EQ(decoded.output_frame.phase_reference_mode, SarPhaseReferenceMode::kCenterBroadside);
  EXPECT_EQ(decoded.output_frame.image_quality_mainlobe_method, SarMainlobeEstimationMethod::k20dB);
  EXPECT_DOUBLE_EQ(decoded.output_frame.range_width_3db_bins, 3.0);
  EXPECT_DOUBLE_EQ(decoded.output_frame.azimuth_width_3db_bins, 5.0);
  EXPECT_DOUBLE_EQ(decoded.output_frame.range_resolution_3db_m, 4.5);
  EXPECT_DOUBLE_EQ(decoded.output_frame.azimuth_resolution_3db_m, 10.0);
  EXPECT_DOUBLE_EQ(decoded.output_frame.image_entropy_nats, 2.25);
  EXPECT_DOUBLE_EQ(decoded.output_frame.image_contrast, 1.5);
  EXPECT_TRUE(decoded.output_frame.has_range_compressed_echo);
  EXPECT_TRUE(decoded.output_frame.has_l3_bp_image);
  EXPECT_FALSE(decoded.output_frame.has_l1_image);
  EXPECT_TRUE(decoded.output_frame.has_image_quality_metrics);
  EXPECT_TRUE(decoded.output_frame.image_resolution_m_valid);
  EXPECT_TRUE(decoded.output_frame.phase_reference_applied);
  ASSERT_EQ(decoded.issues.size(), 1U);
  EXPECT_EQ(decoded.issues[0].severity, SarIssueSeverity::kInfo);
  EXPECT_EQ(decoded.issues[0].phase, SarIssuePhase::kExecution);
  EXPECT_EQ(decoded.issues[0].code, "sar.rda_peak");
  EXPECT_EQ(decoded.issues[0].message, "peak index");
  EXPECT_EQ(decoded.issues[0].location.kind,
            oneq::foundation::ValidationLocationKind::kPlatform);
  EXPECT_EQ(decoded.issues[0].location.entity_index, static_cast<std::size_t>(-1));
  EXPECT_EQ(decoded.issues[0].field, "platform");
  EXPECT_EQ(decoded.status, SarCycleStatus::kCompleted);
  EXPECT_EQ(decoded.raw_phase_history.source, SarRawPhaseHistorySource::kExternalRawIq);
  EXPECT_EQ(decoded.raw_phase_history.pulse_count, 2U);
  EXPECT_EQ(decoded.raw_phase_history.samples_per_pulse, 2U);
  EXPECT_EQ(decoded.raw_phase_history.i_values, result.raw_phase_history.i_values);
  EXPECT_EQ(decoded.raw_phase_history.q_values, result.raw_phase_history.q_values);
  EXPECT_EQ(decoded.focused_image.source, SarFocusedImageSource::kL3Bp);
  EXPECT_EQ(decoded.focused_image.row_count, 2U);
  EXPECT_EQ(decoded.focused_image.column_count, 2U);
  EXPECT_EQ(decoded.focused_image.real_values, result.focused_image.real_values);
  EXPECT_EQ(decoded.focused_image.imaginary_values, result.focused_image.imaginary_values);
  EXPECT_FALSE(decoded.focused_image.is_placeholder);
}

TEST(SarReplayCodecRoundtripTest, CycleResultPreservesFocusedImagePlaceholder) {
  SarCycleResult result;
  result.focused_image.source = SarFocusedImageSource::kL1Rda;
  result.focused_image.row_count = 9U;
  result.focused_image.column_count = 64U;
  result.focused_image.is_placeholder = true;

  SarCycleResult decoded;
  ASSERT_TRUE(DecodeSarCycleResult(EncodeSarCycleResult(result), &decoded));
  EXPECT_EQ(decoded.focused_image.source, SarFocusedImageSource::kL1Rda);
  EXPECT_EQ(decoded.focused_image.row_count, 9U);
  EXPECT_EQ(decoded.focused_image.column_count, 64U);
  EXPECT_TRUE(decoded.focused_image.real_values.empty());
  EXPECT_TRUE(decoded.focused_image.imaginary_values.empty());
  EXPECT_TRUE(decoded.focused_image.is_placeholder);
}

TEST(SarReplayCodecRoundtripTest, CycleResultRejectsMalformedFocusedImage) {
  SarCycleResult malformed;
  malformed.focused_image.source = SarFocusedImageSource::kL1Rda;
  malformed.focused_image.row_count = 2U;
  malformed.focused_image.column_count = 2U;
  malformed.focused_image.real_values = {1.0, 2.0, 3.0};
  malformed.focused_image.imaginary_values = {1.0, 2.0, 3.0, 4.0};
  SarCycleResult decoded;
  EXPECT_FALSE(DecodeSarCycleResult(EncodeSarCycleResult(malformed), &decoded));

  malformed.focused_image.real_values = {1.0, 2.0, 3.0, 4.0};
  malformed.focused_image.real_values[2] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(DecodeSarCycleResult(EncodeSarCycleResult(malformed), &decoded));

  malformed.focused_image.real_values.clear();
  malformed.focused_image.imaginary_values.clear();
  malformed.focused_image.real_values.push_back(1.0);
  malformed.focused_image.imaginary_values.push_back(1.0);
  malformed.focused_image.is_placeholder = true;
  EXPECT_FALSE(DecodeSarCycleResult(EncodeSarCycleResult(malformed), &decoded));
}

TEST(SarReplayCodecRoundtripTest, CycleResultRejectsInvalidFocusedImageStateCombinations) {
  SarCycleResult malformed;
  SarCycleResult decoded;

  malformed.focused_image.source = SarFocusedImageSource::kNone;
  malformed.focused_image.row_count = 1U;
  EXPECT_FALSE(DecodeSarCycleResult(EncodeSarCycleResult(malformed), &decoded));

  malformed.focused_image = SarFocusedImage{};
  malformed.focused_image.is_placeholder = true;
  EXPECT_FALSE(DecodeSarCycleResult(EncodeSarCycleResult(malformed), &decoded));

  malformed.focused_image = SarFocusedImage{};
  malformed.focused_image.source = SarFocusedImageSource::kL1Rda;
  malformed.focused_image.column_count = 2U;
  EXPECT_FALSE(DecodeSarCycleResult(EncodeSarCycleResult(malformed), &decoded));

  malformed.focused_image = SarFocusedImage{};
  malformed.focused_image.source = SarFocusedImageSource::kL3Bp;
  malformed.focused_image.row_count = 2U;
  malformed.focused_image.is_placeholder = true;
  EXPECT_FALSE(DecodeSarCycleResult(EncodeSarCycleResult(malformed), &decoded));

  malformed.focused_image = SarFocusedImage{};
  malformed.focused_image.source = static_cast<SarFocusedImageSource>(99);
  malformed.focused_image.row_count = 1U;
  malformed.focused_image.column_count = 1U;
  malformed.focused_image.real_values.push_back(1.0);
  malformed.focused_image.imaginary_values.push_back(1.0);
  EXPECT_FALSE(DecodeSarCycleResult(EncodeSarCycleResult(malformed), &decoded));
}

TEST(SarReplayCodecRoundtripTest, DecodeCycleResultRejectsUnknownAbortReasonAtomically) {
  // abort_reason 合法值 0..5（kNone..kSensorPoweredOff）；未知值必须 fail closed，
  // 且不得修改调用方传入的输出对象（原子性）。
  const std::int32_t invalid_reasons[] = {6, -1, std::numeric_limits<std::int32_t>::max()};
  for (const std::int32_t invalid_reason : invalid_reasons) {
    SarCycleResult result;
    result.input_cycle_index = 17U;
    result.status = SarCycleStatus::kCompleted;

    EXPECT_FALSE(DecodeSarCycleResult(EncodeCycleResultWithRawAbortReason(invalid_reason),
                                      &result));
    EXPECT_EQ(result.input_cycle_index, 17U);
    EXPECT_EQ(result.status, SarCycleStatus::kCompleted);
  }
}

TEST(SarReplayCodecRoundtripTest, CycleResultDecodeFailureDoesNotModifyOutput) {
  SarCycleResult output;
  output.input_cycle_index = 77U;
  output.focused_image.source = SarFocusedImageSource::kL3Bp;
  output.focused_image.row_count = 1U;
  output.focused_image.column_count = 1U;
  output.focused_image.real_values.push_back(123.0);
  output.focused_image.imaginary_values.push_back(-456.0);
  output.status = SarCycleStatus::kRejectedExecution;
  output.abort_reason = SarPipelineAbortReason::kPipelineExecutionFailed;

  SarCycleResult malformed;
  malformed.input_cycle_index = 9U;
  malformed.raw_phase_history.source = SarRawPhaseHistorySource::kInternallyGenerated;
  malformed.raw_phase_history.pulse_count = 1U;
  malformed.raw_phase_history.samples_per_pulse = 1U;
  malformed.raw_phase_history.i_values.push_back(std::numeric_limits<double>::infinity());
  malformed.raw_phase_history.q_values.push_back(0.0);
  EXPECT_FALSE(DecodeSarCycleResult(EncodeSarCycleResult(malformed), &output));

  EXPECT_EQ(output.input_cycle_index, 77U);
  EXPECT_EQ(output.focused_image.source, SarFocusedImageSource::kL3Bp);
  EXPECT_EQ(output.focused_image.real_values, std::vector<double>({123.0}));
  EXPECT_EQ(output.focused_image.imaginary_values, std::vector<double>({-456.0}));
  EXPECT_EQ(output.status, SarCycleStatus::kRejectedExecution);
  EXPECT_EQ(output.abort_reason, SarPipelineAbortReason::kPipelineExecutionFailed);
}

TEST(SarReplayCodecRoundtripTest, SessionConfigPreservesAllDomains) {
  config::SarSessionConfig config;
  config.sensor_enabled = false;  // 非默认值防 decode 漏读（COMMON-OQ-4 顶层字段）
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
  config.mission.platform_speed_mps = 2.0;
  config.mission.range_sample_count = 64U;
  config.mission.azimuth_pulse_count = 9U;
  config.mission.desired_ground_range_resolution_m = 2.0;
  config.mission.desired_azimuth_resolution_m = 3.0;
  config.mission.l2_velocity_error_stddev_x_mps = 0.5;
  config.mission.l2_velocity_error_stddev_y_mps = 1.5;
  config.mission.l2_velocity_error_stddev_z_mps = 0.25;
  config.mission.l2_random_seed = 2026U;
  config::SarWaypointConfig waypoint;
  waypoint.time_from_session_start_s = 1.25;
  waypoint.latitude_deg = 30.0001;
  waypoint.longitude_deg = 120.0002;
  waypoint.altitude_m = 1500.0;
  config.mission.l3_waypoints.push_back(waypoint);
  config.policy.enable_raw_echo_generation = true;
  config.policy.enable_l1_rda_imaging = true;
  config.policy.enable_l2_motion_compensation = true;
  config.policy.enable_l3_bp_imaging = true;
  config.policy.enable_diagnostics = false;
  config.policy.retain_raw_phase_history = true;
  config.policy.max_allowed_squint_angle_deg = 3.0;
  config.policy.minimum_snr_db = -5.0;
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
  EXPECT_DOUBLE_EQ(decoded.mission.l2_velocity_error_stddev_y_mps, 1.5);
  EXPECT_EQ(decoded.mission.l2_random_seed, 2026U);
  ASSERT_EQ(decoded.mission.l3_waypoints.size(), 1U);
  EXPECT_DOUBLE_EQ(decoded.mission.l3_waypoints[0].time_from_session_start_s, 1.25);
  EXPECT_DOUBLE_EQ(decoded.mission.l3_waypoints[0].longitude_deg, 120.0002);
  EXPECT_TRUE(decoded.policy.enable_l1_rda_imaging);
  EXPECT_TRUE(decoded.policy.enable_l2_motion_compensation);
  EXPECT_TRUE(decoded.policy.enable_l3_bp_imaging);
  EXPECT_TRUE(decoded.policy.retain_raw_phase_history);
  EXPECT_DOUBLE_EQ(decoded.policy.minimum_snr_db, -5.0);
  EXPECT_FALSE(decoded.environment.use_flat_earth_geometry);
  EXPECT_TRUE(decoded.environment.enable_atmospheric_attenuation);
  EXPECT_FALSE(decoded.sensor_enabled);
}

TEST(SarReplayCodecRoundtripTest, CycleResultRejectsMalformedRawPhaseHistory) {
  SarCycleResult malformed;
  malformed.raw_phase_history.source = SarRawPhaseHistorySource::kInternallyGenerated;
  malformed.raw_phase_history.pulse_count = 2U;
  malformed.raw_phase_history.samples_per_pulse = 2U;
  malformed.raw_phase_history.i_values = {1.0, 2.0, 3.0};
  malformed.raw_phase_history.q_values = {1.0, 2.0, 3.0, 4.0};
  SarCycleResult decoded;
  EXPECT_FALSE(DecodeSarCycleResult(EncodeSarCycleResult(malformed), &decoded));

  malformed.raw_phase_history.i_values = {1.0, 2.0, 3.0, 4.0};
  malformed.raw_phase_history.i_values[2] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(DecodeSarCycleResult(EncodeSarCycleResult(malformed), &decoded));
}

TEST(SarReplayCodecRoundtripTest, RuntimeConfigPatchPreservesHasBitsAndValues) {
  // 通过 SarRuntimeConfigBuilder 构造补丁（与 EOS/ESR/AR 对齐，避免手写 has_*）。
  const config::SarRuntimeConfigPatch patch =
      config::SarRuntimeConfigBuilder()
          .WithEnableRawEchoGeneration(false)
          .WithEnableL1RdaImaging(true)
          .WithRetainRawPhaseHistory(true)
          .WithMinimumSnrDb(6.5)
          .WithSensorEnabled(false)
          .Build();

  const std::string bytes = EncodeSarRuntimeConfigPatch(patch);
  ASSERT_FALSE(bytes.empty());
  config::SarRuntimeConfigPatch decoded;
  ASSERT_TRUE(DecodeSarRuntimeConfigPatch(bytes, &decoded));

  EXPECT_TRUE(decoded.has_enable_raw_echo_generation);
  EXPECT_FALSE(decoded.enable_raw_echo_generation);
  EXPECT_TRUE(decoded.has_enable_l1_rda_imaging);
  EXPECT_TRUE(decoded.enable_l1_rda_imaging);
  EXPECT_TRUE(decoded.has_retain_raw_phase_history);
  EXPECT_TRUE(decoded.retain_raw_phase_history);
  EXPECT_TRUE(decoded.has_minimum_snr_db);
  EXPECT_DOUBLE_EQ(decoded.minimum_snr_db, 6.5);
  EXPECT_TRUE(decoded.has_sensor_enabled);
  EXPECT_FALSE(decoded.sensor_enabled);
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
