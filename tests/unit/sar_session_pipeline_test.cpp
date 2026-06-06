#include <gtest/gtest.h>

#include "1q/sar/session/SarSessionFactory.h"

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

TEST(SarSessionPipelineTest, StepWithResultRunsRawRangeAndRdaPipeline) {
  session::SarSession session = session::SarSessionFactory::Create(MakeSmallRdaConfig());

  const session::SarCycleResult result = session.StepWithResult(MakeInput());

  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.has_error);
  EXPECT_FALSE(result.reused_previous_output);
  EXPECT_TRUE(result.output_frame.has_raw_echo);
  EXPECT_TRUE(result.output_frame.has_range_compressed_echo);
  EXPECT_TRUE(result.output_frame.has_l1_image);
  EXPECT_EQ(result.output_frame.completed_stage, session::SarProcessingStage::kL1RdaImage);
  EXPECT_EQ(result.output_frame.range_sample_count, 64U);
  EXPECT_EQ(result.output_frame.azimuth_pulse_count, 9U);
  EXPECT_FALSE(result.diagnostics.empty());
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.rda_peak", "image_entropy_nats="));
  EXPECT_TRUE(HasDiagnosticContaining(result, "sar.rda_peak", "azimuth_width_3db_bins="));
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
}

}  // namespace
}  // namespace sar
