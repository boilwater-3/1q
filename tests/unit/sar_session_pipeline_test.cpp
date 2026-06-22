#include <gtest/gtest.h>

#include <limits>

#include "1q/sar/session/SarSessionFactory.h"
#include "1q/sar/session/SarTraceSession.h"
#include "sar/session/SarReplayFlatbufferCodec.h"

namespace sar {
namespace {

bool HasDiagnosticContaining(const session::SarCycleResult& result, const std::string& code,
                             const std::string& text) {
  for (const session::SarDiagnosticIssue& diagnostic : result.diagnostics) {
    if (diagnostic.code == code && diagnostic.message.find(text) != std::string::npos) {
      return true;
    }
  }
  return false;
}

bool HasNonZeroFocusedPixel(const session::SarFocusedImage& image) {
  for (std::size_t index = 0U; index < image.real_values.size(); ++index) {
    if (image.real_values[index] != 0.0 || image.imaginary_values[index] != 0.0) {
      return true;
    }
  }
  return false;
}

config::SarSessionConfig MakeSmallRdaConfig() {
  config::SarSessionConfig config;
  config.hardware.carrier_frequency_hz = 1.0e9;
  config.hardware.bandwidth_hz = 25.0e6;
  config.hardware.pulse_width_s = 0.16e-6;
  config.hardware.pulse_repetition_frequency_hz = 20.0;
  config.hardware.sample_rate_hz = 100.0e6;
  config.mission.nominal_slant_range_m = 29.9792458;
  config.mission.platform_speed_mps = 2.0;
  config.mission.range_sample_count = 64U;
  config.mission.azimuth_pulse_count = 9U;
  config.policy.enable_l1_rda_imaging = true;
  return config;
}

config::SarSessionConfig MakeSmallL3BpConfig() {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.enable_l1_rda_imaging = false;
  config.policy.enable_l3_bp_imaging = true;
  const double meters_to_degrees = 180.0 / (3.14159265358979323846 * 6378137.0);
  config::SarWaypointConfig start;
  start.time_from_session_start_s = 0.0;
  start.longitude_deg = -0.4 * meters_to_degrees;
  config::SarWaypointConfig turn;
  turn.time_from_session_start_s = 0.2;
  config::SarWaypointConfig end;
  end.time_from_session_start_s = 1.0;
  end.longitude_deg = 1.6 * meters_to_degrees;
  end.latitude_deg = 3.0 * meters_to_degrees;
  config.mission.l3_waypoints = {start, turn, end};
  return config;
}

session::SarCycleInput MakeInput(std::uint32_t cycle_index = 1U) {
  session::SarCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 0.1;
  input.platform.latitude_deg = 0.0;
  input.platform.longitude_deg = 0.0;
  input.platform.altitude_m = 0.0;

  session::SarPointTarget target;
  target.latitude_deg = 29.9792458 / 6378137.0 * 180.0 / 3.14159265358979323846;
  target.longitude_deg = 0.0;
  target.altitude_m = 0.0;
  target.radar_cross_section_dbsm = 80.0;
  input.point_targets.push_back(target);
  return input;
}

session::SarCycleInput MakeExternalRawIqInput() {
  session::SarCycleInput input = MakeInput();
  input.raw_iq.pulse_count = 9U;
  input.raw_iq.samples_per_pulse = 64U;
  input.raw_iq.i_values.assign(9U * 64U, 0.0);
  input.raw_iq.q_values.assign(9U * 64U, 0.0);
  for (std::size_t row = 0U; row < 9U; ++row) {
    input.raw_iq.i_values[row * 64U + 20U] = 1.0;
  }
  return input;
}

session::SarCycleInput MakeExternalRawIqInputWithTrajectory() {
  session::SarCycleInput input = MakeExternalRawIqInput();
  for (std::size_t index = 0U; index < input.raw_iq.pulse_count; ++index) {
    session::SarRawIqFrame::PulseState state;
    state.pulse_id = static_cast<std::uint64_t>(index);
    state.time_s = static_cast<double>(index) / 20.0;
    state.position_x_m = -0.4 + 0.1 * static_cast<double>(index);
    state.velocity_x_mps = 2.0;
    input.raw_iq.pulse_states.push_back(state);
  }
  return input;
}

session::SarCycleInput MakeExternalRawIqInputWithDualTrajectory() {
  session::SarCycleInput input = MakeExternalRawIqInputWithTrajectory();
  input.raw_iq.ideal_pulse_states = input.raw_iq.pulse_states;
  for (session::SarRawIqFrame::PulseState& state : input.raw_iq.pulse_states) {
    state.position_y_m += 0.25;
  }
  return input;
}

TEST(SarSessionPipelineTest, StepWithResultRunsRawRangeAndRdaPipeline) {
  session::SarSession session = session::SarSessionFactory::Create(MakeSmallRdaConfig());

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.has_error);
  EXPECT_FALSE(result.reused_previous_output);
  EXPECT_TRUE(result.output_frame.has_raw_echo);
  EXPECT_TRUE(result.output_frame.has_range_compressed_echo);
  EXPECT_TRUE(result.output_frame.has_l1_image);
  EXPECT_TRUE(result.output_frame.has_image_quality_metrics);
  EXPECT_TRUE(result.output_frame.image_resolution_m_valid);
  EXPECT_TRUE(result.output_frame.phase_reference_applied);
  EXPECT_EQ(result.output_frame.phase_reference_mode,
            session::SarPhaseReferenceMode::kCenterBroadside);
  EXPECT_EQ(result.output_frame.image_quality_mainlobe_method,
            session::SarMainlobeEstimationMethod::k3dB);
  EXPECT_GT(result.output_frame.range_width_3db_bins, 0.0);
  EXPECT_GT(result.output_frame.azimuth_width_3db_bins, 0.0);
  EXPECT_GT(result.output_frame.range_resolution_3db_m, 0.0);
  EXPECT_GT(result.output_frame.azimuth_resolution_3db_m, 0.0);
  EXPECT_GE(result.output_frame.image_entropy_nats, 0.0);
  EXPECT_GE(result.output_frame.image_contrast, 0.0);
  EXPECT_EQ(result.output_frame.completed_stage, session::SarProcessingStage::kL1RdaImage);
  EXPECT_EQ(result.output_frame.range_sample_count, 64U);
  EXPECT_EQ(result.output_frame.azimuth_pulse_count, 9U);
  EXPECT_EQ(result.focused_image.source, session::SarFocusedImageSource::kL1Rda);
  EXPECT_EQ(result.focused_image.row_count, 9U);
  EXPECT_EQ(result.focused_image.column_count, 64U);
  EXPECT_EQ(result.focused_image.real_values.size(), 9U * 64U);
  EXPECT_EQ(result.focused_image.imaginary_values.size(), 9U * 64U);
  EXPECT_FALSE(result.focused_image.is_placeholder);
  EXPECT_TRUE(HasNonZeroFocusedPixel(result.focused_image));
  EXPECT_FALSE(result.diagnostics.empty());
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.rda_peak", "image_entropy_nats="));
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.rda_peak", "image_contrast="));
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.rda_peak", "phase_reference_mode="));
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.rda_peak", "phase_reference_applied=1"));
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.rda_peak", "range_width_3db_bins="));
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.rda_peak", "azimuth_width_3db_bins="));
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.rda_peak", "range_resolution_3db_m="));
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.rda_peak", "azimuth_resolution_3db_m="));
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.rda_peak", "azimuth_sample_spacing_m="));
  EXPECT_TRUE(
      HasDiagnosticContaining(result, "sar.rda_peak", "azimuth_phase_curvature_rad_per_pulse2="));
  EXPECT_TRUE(
      HasDiagnosticContaining(result, "sar.rda_peak", "azimuth_quadratic_phase_span_rad="));
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.rda_peak", "max_geometric_doppler_hz="));
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.rda_peak", "doppler_nyquist_margin="));
}

TEST(SarSessionPipelineTest, RetainFocusedImageFalseProducesPlaceholder) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.retain_focused_image = false;
  session::SarSession session = session::SarSessionFactory::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.has_error);
  EXPECT_TRUE(result.output_frame.has_l1_image);
  // 元数据仍完整，但像素数据被跳过。
  EXPECT_EQ(result.focused_image.source, session::SarFocusedImageSource::kL1Rda);
  EXPECT_EQ(result.focused_image.row_count, 9U);
  EXPECT_EQ(result.focused_image.column_count, 64U);
  EXPECT_TRUE(result.focused_image.is_placeholder);
  EXPECT_TRUE(result.focused_image.real_values.empty());
  EXPECT_TRUE(result.focused_image.imaginary_values.empty());
}

TEST(SarSessionPipelineTest, RetainFocusedImageFalseAppliesToL3Bp) {
  config::SarSessionConfig config = MakeSmallL3BpConfig();
  config.policy.retain_focused_image = false;
  session::SarSession session = session::SarSessionFactory::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.has_error);
  EXPECT_TRUE(result.output_frame.has_l3_bp_image);
  EXPECT_EQ(result.focused_image.source, session::SarFocusedImageSource::kL3Bp);
  EXPECT_TRUE(result.focused_image.is_placeholder);
  EXPECT_TRUE(result.focused_image.real_values.empty());
}

TEST(SarSessionPipelineTest, RawPulseHistoryUsesCrossCycleRingBuffer) {
  session::SarSession session = session::SarSessionFactory::Create(MakeSmallRdaConfig());

  const session::SarCycleResult first = session.StepWithResult(MakeInput(1U));
  ASSERT_TRUE(first.executed_this_cycle);
  EXPECT_TRUE(first.output_frame.has_l1_image);
  EXPECT_TRUE(HasDiagnosticContaining(first, "sar.pulse_ring_buffer", "generated=9"));

  const session::SarCycleResult second = session.StepWithResult(MakeInput(2U));
  EXPECT_TRUE(second.executed_this_cycle);
  EXPECT_TRUE(second.output_frame.has_l1_image);
  EXPECT_TRUE(HasDiagnosticContaining(second, "sar.pulse_ring_buffer", "generated=2"));
  EXPECT_TRUE(HasDiagnosticContaining(second, "sar.pulse_ring_buffer", "overflow=true"));
}

TEST(SarSessionPipelineTest, ExternalRawIqRunsL1RdaAndReturnsFocusedImage) {
  session::SarSession session = session::SarSessionFactory::Create(MakeSmallRdaConfig());

  const session::SarCycleResult result = session.StepWithResult(MakeExternalRawIqInput());

  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.has_error);
  EXPECT_TRUE(result.output_frame.has_raw_echo);
  EXPECT_TRUE(result.output_frame.has_l1_image);
  EXPECT_EQ(result.focused_image.source, session::SarFocusedImageSource::kL1Rda);
  EXPECT_TRUE(HasNonZeroFocusedPixel(result.focused_image));
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.external_raw_iq", "pulses=9"));
  EXPECT_FALSE(HasDiagnosticContaining(result, "sar.pulse_ring_buffer", ""));
}

TEST(SarSessionPipelineTest, TraceSessionWithoutReplayWriterAcceptsExternalRawIq) {
  session::SarTraceSession session(MakeSmallRdaConfig());

  const session::SarCycleResult result = session.StepWithResult(MakeExternalRawIqInput());

  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.has_error);
  EXPECT_TRUE(result.output_frame.has_l1_image);
  EXPECT_EQ(result.focused_image.source, session::SarFocusedImageSource::kL1Rda);
}

TEST(SarSessionPipelineTest, SummaryReplayCodecRejectsExternalRawIq) {
  EXPECT_TRUE(session::EncodeSarCycleInput(MakeExternalRawIqInput()).empty());
}

TEST(SarSessionPipelineTest, ExternalRawIqRejectsShapeMismatchAndAdvancedPaths) {
  session::SarCycleInput malformed = MakeExternalRawIqInput();
  malformed.raw_iq.i_values.pop_back();
  session::SarSession malformed_session = session::SarSessionFactory::Create(MakeSmallRdaConfig());
  const session::SarCycleResult malformed_result = malformed_session.StepWithResult(malformed);
  EXPECT_FALSE(malformed_result.executed_this_cycle);
  EXPECT_EQ(malformed_result.abort_reason, "external_raw_iq_shape_mismatch");

  session::SarCycleInput non_finite = MakeExternalRawIqInput();
  non_finite.raw_iq.q_values[0] = std::numeric_limits<double>::quiet_NaN();
  session::SarSession non_finite_session = session::SarSessionFactory::Create(MakeSmallRdaConfig());
  const session::SarCycleResult non_finite_result =
      non_finite_session.StepWithResult(non_finite);
  EXPECT_FALSE(non_finite_result.executed_this_cycle);
  EXPECT_EQ(non_finite_result.abort_reason, "external_raw_iq_non_finite");

  config::SarSessionConfig l2_config = MakeSmallRdaConfig();
  l2_config.policy.enable_l2_motion_compensation = true;
  session::SarSession l2_session = session::SarSessionFactory::Create(l2_config);
  const session::SarCycleResult l2_result = l2_session.StepWithResult(MakeExternalRawIqInput());
  EXPECT_FALSE(l2_result.executed_this_cycle);
  EXPECT_EQ(l2_result.abort_reason, "external_raw_iq_trajectory_required");
}

TEST(SarSessionPipelineTest, ExternalRawIqWithPulseStatesRunsL3Bp) {
  session::SarSession session = session::SarSessionFactory::Create(MakeSmallL3BpConfig());

  const session::SarCycleResult result =
      session.StepWithResult(MakeExternalRawIqInputWithTrajectory());

  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.has_error);
  EXPECT_TRUE(result.output_frame.has_l3_bp_image);
  EXPECT_EQ(result.focused_image.source, session::SarFocusedImageSource::kL3Bp);
  EXPECT_TRUE(HasNonZeroFocusedPixel(result.focused_image));
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.external_raw_iq", "pulses=9"));
  EXPECT_FALSE(HasDiagnosticContaining(result, "sar.l3_trajectory", ""));
}

TEST(SarSessionPipelineTest, ExternalRawIqL1ExplicitlyIgnoresPulseStates) {
  session::SarSession session = session::SarSessionFactory::Create(MakeSmallRdaConfig());

  const session::SarCycleResult result =
      session.StepWithResult(MakeExternalRawIqInputWithTrajectory());

  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.has_error);
  EXPECT_TRUE(result.output_frame.has_l1_image);
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.external_raw_iq_trajectory_ignored", ""));
}

TEST(SarSessionPipelineTest, ExternalRawIqDualTrajectoryRunsL2MotionCompensation) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.enable_l2_motion_compensation = true;
  session::SarSession session = session::SarSessionFactory::Create(config);

  const session::SarCycleResult result =
      session.StepWithResult(MakeExternalRawIqInputWithDualTrajectory());

  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.has_error);
  EXPECT_TRUE(result.output_frame.has_l1_image);
  EXPECT_EQ(result.focused_image.source, session::SarFocusedImageSource::kL1Rda);
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.motion_compensation", "max_abs_range_error_m="));
  EXPECT_FALSE(HasDiagnosticContaining(result, "sar.external_raw_iq_trajectory_ignored", ""));
}

TEST(SarSessionPipelineTest, ExternalRawIqL2RejectsMissingOrInvalidIdealTrajectory) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.enable_l2_motion_compensation = true;

  session::SarSession missing_session = session::SarSessionFactory::Create(config);
  const session::SarCycleResult missing_result =
      missing_session.StepWithResult(MakeExternalRawIqInputWithTrajectory());
  EXPECT_FALSE(missing_result.executed_this_cycle);
  EXPECT_EQ(missing_result.abort_reason, "external_raw_iq_ideal_trajectory_required");

  session::SarCycleInput invalid = MakeExternalRawIqInputWithDualTrajectory();
  invalid.raw_iq.ideal_pulse_states[3].time_s =
      invalid.raw_iq.ideal_pulse_states[2].time_s;
  session::SarSession invalid_session = session::SarSessionFactory::Create(config);
  const session::SarCycleResult invalid_result = invalid_session.StepWithResult(invalid);
  EXPECT_FALSE(invalid_result.executed_this_cycle);
  EXPECT_EQ(invalid_result.abort_reason, "external_raw_iq_invalid_ideal_trajectory");
}

TEST(SarSessionPipelineTest, ExternalRawIqBpRejectsMissingOrInvalidTrajectory) {
  session::SarSession missing_session = session::SarSessionFactory::Create(MakeSmallL3BpConfig());
  const session::SarCycleResult missing_result =
      missing_session.StepWithResult(MakeExternalRawIqInput());
  EXPECT_FALSE(missing_result.executed_this_cycle);
  EXPECT_EQ(missing_result.abort_reason, "external_raw_iq_trajectory_required");

  session::SarCycleInput invalid = MakeExternalRawIqInputWithTrajectory();
  invalid.raw_iq.pulse_states[2].pulse_id = invalid.raw_iq.pulse_states[1].pulse_id;
  session::SarSession invalid_session = session::SarSessionFactory::Create(MakeSmallL3BpConfig());
  const session::SarCycleResult invalid_result = invalid_session.StepWithResult(invalid);
  EXPECT_FALSE(invalid_result.executed_this_cycle);
  EXPECT_EQ(invalid_result.abort_reason, "external_raw_iq_invalid_trajectory");

  session::SarCycleInput non_finite = MakeExternalRawIqInputWithTrajectory();
  non_finite.raw_iq.pulse_states[0].position_y_m =
      std::numeric_limits<double>::infinity();
  session::SarSession non_finite_session =
      session::SarSessionFactory::Create(MakeSmallL3BpConfig());
  const session::SarCycleResult non_finite_result =
      non_finite_session.StepWithResult(non_finite);
  EXPECT_FALSE(non_finite_result.executed_this_cycle);
  EXPECT_EQ(non_finite_result.abort_reason, "external_raw_iq_invalid_trajectory");
}

TEST(SarSessionPipelineTest, RuntimeSizeGateRejectsUnapprovedLargeRda) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.mission.range_sample_count = 2048U;
  config.mission.azimuth_pulse_count = 1024U;
  session::SarSession session = session::SarSessionFactory::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_FALSE(result.executed_this_cycle);
  EXPECT_TRUE(result.has_error);
  EXPECT_EQ(result.abort_reason, "rda_size_gate");
  EXPECT_FALSE(result.output_frame.has_l1_image);
}

TEST(SarSessionPipelineTest, RdaRequiresRawEchoGenerationInPhase1Pipeline) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.enable_raw_echo_generation = false;
  session::SarSession session = session::SarSessionFactory::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_FALSE(result.executed_this_cycle);
  EXPECT_TRUE(result.has_error);
  EXPECT_EQ(result.abort_reason, "rda_requires_raw_echo");
  EXPECT_FALSE(result.output_frame.has_l1_image);
}

TEST(SarSessionPipelineTest, L2MotionCompensationIsDefaultOffAndRunsWhenExplicitlyEnabled) {
  config::SarSessionConfig l1_config = MakeSmallRdaConfig();
  EXPECT_FALSE(l1_config.policy.enable_l2_motion_compensation);
  session::SarSession l1_session = session::SarSessionFactory::Create(l1_config);
  const session::SarCycleResult l1_result = l1_session.StepWithResult(MakeInput());
  ASSERT_TRUE(l1_result.executed_this_cycle);
  EXPECT_FALSE(HasDiagnosticContaining(l1_result, "sar.l2_trajectory", ""));
  EXPECT_FALSE(HasDiagnosticContaining(l1_result, "sar.motion_compensation", ""));

  config::SarSessionConfig l2_config = MakeSmallRdaConfig();
  l2_config.policy.enable_l2_motion_compensation = true;
  l2_config.mission.l2_velocity_error_stddev_y_mps = 30.0;
  l2_config.mission.l2_velocity_error_stddev_z_mps = 10.0;
  l2_config.mission.l2_random_seed = 2026U;
  session::SarSession l2_session = session::SarSessionFactory::Create(l2_config);
  const session::SarCycleResult l2_result = l2_session.StepWithResult(MakeInput());

  EXPECT_TRUE(l2_result.executed_this_cycle);
  EXPECT_FALSE(l2_result.has_error);
  EXPECT_TRUE(l2_result.output_frame.has_l1_image);
  EXPECT_TRUE(HasDiagnosticContaining(l2_result, "sar.l2_trajectory", "max_position_error_m="));
  EXPECT_TRUE(
      HasDiagnosticContaining(l2_result, "sar.motion_compensation", "max_abs_range_error_m="));
}

TEST(SarSessionPipelineTest, ZeroPerturbationL2StrictlyDegradesToL1Trajectory) {
  session::SarSession l1_session = session::SarSessionFactory::Create(MakeSmallRdaConfig());
  const session::SarCycleResult l1_result = l1_session.StepWithResult(MakeInput());
  ASSERT_TRUE(l1_result.executed_this_cycle);

  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.enable_l2_motion_compensation = true;
  session::SarSession session = session::SarSessionFactory::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  ASSERT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.has_error);
  EXPECT_TRUE(result.output_frame.has_l1_image);
  EXPECT_EQ(result.output_frame.cycle_index, l1_result.output_frame.cycle_index);
  EXPECT_EQ(result.output_frame.completed_stage, l1_result.output_frame.completed_stage);
  EXPECT_EQ(result.output_frame.range_sample_count, l1_result.output_frame.range_sample_count);
  EXPECT_EQ(result.output_frame.azimuth_pulse_count, l1_result.output_frame.azimuth_pulse_count);
  EXPECT_EQ(result.output_frame.center_slant_range_m, l1_result.output_frame.center_slant_range_m);
  EXPECT_EQ(result.output_frame.estimated_snr_db, l1_result.output_frame.estimated_snr_db);
  EXPECT_EQ(result.output_frame.has_raw_echo, l1_result.output_frame.has_raw_echo);
  EXPECT_EQ(result.output_frame.has_range_compressed_echo,
            l1_result.output_frame.has_range_compressed_echo);
  EXPECT_EQ(result.output_frame.has_l1_image, l1_result.output_frame.has_l1_image);
  EXPECT_TRUE(
      HasDiagnosticContaining(result, "sar.l2_trajectory", "max_position_error_m=0.000000"));
  EXPECT_TRUE(
      HasDiagnosticContaining(result, "sar.l2_trajectory", "rms_position_error_m=0.000000"));
  EXPECT_TRUE(
      HasDiagnosticContaining(result, "sar.motion_compensation", "max_abs_range_error_m=0.000000"));
  EXPECT_TRUE(
      HasDiagnosticContaining(result, "sar.motion_compensation", "rms_range_error_m=0.000000"));
}

TEST(SarSessionPipelineTest, L2TrajectoryHistoryRemainsAlignedAcrossCycles) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.enable_l2_motion_compensation = true;
  config.mission.l2_velocity_error_stddev_y_mps = 30.0;
  config.mission.l2_velocity_error_stddev_z_mps = 10.0;
  config.mission.l2_random_seed = 2026U;
  session::SarSession session = session::SarSessionFactory::Create(config);

  const session::SarCycleResult first = session.StepWithResult(MakeInput(1U));
  ASSERT_TRUE(first.executed_this_cycle);
  EXPECT_FALSE(first.has_error);
  EXPECT_TRUE(HasDiagnosticContaining(first, "sar.pulse_ring_buffer", "generated=9"));
  EXPECT_TRUE(HasDiagnosticContaining(first, "sar.motion_compensation", ""));

  const session::SarCycleResult second = session.StepWithResult(MakeInput(2U));
  EXPECT_TRUE(second.executed_this_cycle);
  EXPECT_FALSE(second.has_error);
  EXPECT_TRUE(second.output_frame.has_l1_image);
  EXPECT_TRUE(HasDiagnosticContaining(second, "sar.pulse_ring_buffer", "generated=2"));
  EXPECT_TRUE(HasDiagnosticContaining(second, "sar.pulse_ring_buffer", "overflow=true"));
  EXPECT_TRUE(HasDiagnosticContaining(second, "sar.l2_trajectory", "max_position_error_m="));
  EXPECT_TRUE(HasDiagnosticContaining(second, "sar.motion_compensation", "max_abs_range_error_m="));
}

TEST(SarSessionPipelineTest, L2MotionCompensationRequiresRawEchoAndRda) {
  config::SarSessionConfig config = MakeSmallRdaConfig();
  config.policy.enable_l2_motion_compensation = true;
  config.policy.enable_l1_rda_imaging = false;
  session::SarSession session = session::SarSessionFactory::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_FALSE(result.executed_this_cycle);
  EXPECT_TRUE(result.has_error);
  EXPECT_EQ(result.abort_reason, "invalid_l2_motion_compensation_config");
}

TEST(SarSessionPipelineTest, L3BpRunsOnlyWhenExplicitlyEnabled) {
  session::SarSession session = session::SarSessionFactory::Create(MakeSmallL3BpConfig());

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.has_error);
  EXPECT_TRUE(result.output_frame.has_raw_echo);
  EXPECT_TRUE(result.output_frame.has_range_compressed_echo);
  EXPECT_FALSE(result.output_frame.has_l1_image);
  EXPECT_TRUE(result.output_frame.has_l3_bp_image);
  EXPECT_TRUE(result.output_frame.has_image_quality_metrics);
  EXPECT_FALSE(result.output_frame.image_resolution_m_valid);
  EXPECT_FALSE(result.output_frame.phase_reference_applied);
  EXPECT_EQ(result.output_frame.phase_reference_mode, session::SarPhaseReferenceMode::kNative);
  EXPECT_EQ(result.output_frame.image_quality_mainlobe_method,
            session::SarMainlobeEstimationMethod::k3dB);
  EXPECT_GT(result.output_frame.range_width_3db_bins, 0.0);
  EXPECT_GT(result.output_frame.azimuth_width_3db_bins, 0.0);
  EXPECT_GE(result.output_frame.image_entropy_nats, 0.0);
  EXPECT_GE(result.output_frame.image_contrast, 0.0);
  EXPECT_EQ(result.output_frame.completed_stage, session::SarProcessingStage::kL3BpImage);
  EXPECT_EQ(result.focused_image.source, session::SarFocusedImageSource::kL3Bp);
  EXPECT_EQ(result.focused_image.row_count, 9U);
  EXPECT_EQ(result.focused_image.column_count, 64U);
  EXPECT_EQ(result.focused_image.real_values.size(), 9U * 64U);
  EXPECT_EQ(result.focused_image.imaginary_values.size(), 9U * 64U);
  EXPECT_FALSE(result.focused_image.is_placeholder);
  EXPECT_TRUE(HasNonZeroFocusedPixel(result.focused_image));
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.l3_trajectory", "generated=9"));
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.bp_peak", "peak_row="));
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.bp_traversal", "pulse_major"));
}

TEST(SarSessionPipelineTest, L3BpRejectsMutualExclusionAndSizeViolations) {
  config::SarSessionConfig mutual_exclusion = MakeSmallL3BpConfig();
  mutual_exclusion.policy.enable_l1_rda_imaging = true;
  session::SarSession mutual_session = session::SarSessionFactory::Create(mutual_exclusion);
  const session::SarCycleResult mutual_result = mutual_session.StepWithResult(MakeInput());
  EXPECT_FALSE(mutual_result.executed_this_cycle);
  EXPECT_EQ(mutual_result.abort_reason, "invalid_l3_bp_config");

  config::SarSessionConfig oversized = MakeSmallL3BpConfig();
  oversized.mission.range_sample_count = 129U;
  session::SarSession oversized_session = session::SarSessionFactory::Create(oversized);
  const session::SarCycleResult oversized_result = oversized_session.StepWithResult(MakeInput());
  EXPECT_FALSE(oversized_result.executed_this_cycle);
  EXPECT_EQ(oversized_result.abort_reason, "l3_bp_size_gate");
}

TEST(SarSessionPipelineTest, L3BpRequiresRawEchoAndRangeCompression) {
  config::SarSessionConfig no_raw = MakeSmallL3BpConfig();
  no_raw.policy.enable_raw_echo_generation = false;
  session::SarSession no_raw_session = session::SarSessionFactory::Create(no_raw);
  const session::SarCycleResult no_raw_result = no_raw_session.StepWithResult(MakeInput());
  EXPECT_FALSE(no_raw_result.executed_this_cycle);
  EXPECT_EQ(no_raw_result.abort_reason, "invalid_l3_bp_config");

  config::SarSessionConfig no_range_compression = MakeSmallL3BpConfig();
  no_range_compression.policy.enable_range_compression = false;
  session::SarSession no_range_session = session::SarSessionFactory::Create(no_range_compression);
  const session::SarCycleResult no_range_result = no_range_session.StepWithResult(MakeInput());
  EXPECT_FALSE(no_range_result.executed_this_cycle);
  EXPECT_EQ(no_range_result.abort_reason, "invalid_l3_bp_config");
}

TEST(SarSessionPipelineTest, L3BpRejectsInvalidWaypointStructure) {
  config::SarSessionConfig nonzero_start = MakeSmallL3BpConfig();
  nonzero_start.mission.l3_waypoints.front().time_from_session_start_s = 0.01;
  session::SarSession nonzero_start_session = session::SarSessionFactory::Create(nonzero_start);
  const session::SarCycleResult nonzero_start_result =
      nonzero_start_session.StepWithResult(MakeInput());
  EXPECT_FALSE(nonzero_start_result.executed_this_cycle);
  EXPECT_EQ(nonzero_start_result.abort_reason, "invalid_l3_bp_config");

  config::SarSessionConfig nonmonotonic = MakeSmallL3BpConfig();
  nonmonotonic.mission.l3_waypoints.back().time_from_session_start_s =
      nonmonotonic.mission.l3_waypoints[1].time_from_session_start_s;
  session::SarSession nonmonotonic_session = session::SarSessionFactory::Create(nonmonotonic);
  const session::SarCycleResult nonmonotonic_result =
      nonmonotonic_session.StepWithResult(MakeInput());
  EXPECT_FALSE(nonmonotonic_result.executed_this_cycle);
  EXPECT_EQ(nonmonotonic_result.abort_reason, "invalid_l3_bp_config");
}

TEST(SarSessionPipelineTest, L3BpTrajectoryHistoryRemainsAlignedAcrossCycles) {
  session::SarSession session = session::SarSessionFactory::Create(MakeSmallL3BpConfig());

  const session::SarCycleResult first = session.StepWithResult(MakeInput(1U));
  ASSERT_TRUE(first.executed_this_cycle);
  ASSERT_FALSE(first.has_error);
  EXPECT_TRUE(first.output_frame.has_l3_bp_image);
  EXPECT_TRUE(HasDiagnosticContaining(first, "sar.l3_trajectory", "generated=9"));
  EXPECT_TRUE(HasDiagnosticContaining(first, "sar.l3_trajectory", "last_time_s=0.400000"));

  const session::SarCycleResult second = session.StepWithResult(MakeInput(2U));
  EXPECT_TRUE(second.executed_this_cycle);
  EXPECT_FALSE(second.has_error);
  EXPECT_TRUE(second.output_frame.has_l3_bp_image);
  EXPECT_TRUE(HasDiagnosticContaining(second, "sar.pulse_ring_buffer", "generated=2"));
  EXPECT_TRUE(HasDiagnosticContaining(second, "sar.pulse_ring_buffer", "overflow=true"));
  EXPECT_TRUE(HasDiagnosticContaining(second, "sar.l3_trajectory", "first_time_s=0.450000"));
  EXPECT_TRUE(HasDiagnosticContaining(second, "sar.l3_trajectory", "last_time_s=0.500000"));
  EXPECT_TRUE(HasDiagnosticContaining(second, "sar.bp_traversal", "pulse_major"));
}

TEST(SarSessionPipelineTest, L3BpRejectsWaypointCoverageGap) {
  config::SarSessionConfig config = MakeSmallL3BpConfig();
  config.mission.l3_waypoints.back().time_from_session_start_s = 0.2;
  config.mission.l3_waypoints[1].time_from_session_start_s = 0.1;
  session::SarSession session = session::SarSessionFactory::Create(config);

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_FALSE(result.executed_this_cycle);
  EXPECT_TRUE(result.has_error);
  EXPECT_EQ(result.abort_reason, "l3_waypoint_coverage");
}

TEST(SarSessionPipelineTest, InvalidCycleReusesPreviousOutput) {
  session::SarSession session = session::SarSessionFactory::Create(MakeSmallRdaConfig());

  const session::SarCycleResult first = session.StepWithResult(MakeInput(3U));
  ASSERT_TRUE(first.executed_this_cycle);
  session::SarCycleInput invalid = MakeInput(4U);
  invalid.dt_sec = 0.0;
  const session::SarCycleResult second = session.StepWithResult(invalid);

  EXPECT_FALSE(second.executed_this_cycle);
  EXPECT_TRUE(second.has_error);
  EXPECT_TRUE(second.reused_previous_output);
  EXPECT_EQ(second.output_frame.cycle_index, first.output_frame.cycle_index);
  EXPECT_TRUE(second.output_frame.has_l1_image);
  EXPECT_EQ(second.focused_image.source, session::SarFocusedImageSource::kNone);
  EXPECT_TRUE(second.focused_image.real_values.empty());
  EXPECT_TRUE(second.focused_image.imaginary_values.empty());
}

}  // namespace
}  // namespace sar
