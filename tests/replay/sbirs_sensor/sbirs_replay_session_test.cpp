#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <sstream>
#include <string>

#include "1q/replay/ReplayTrace.h"
#include "1q/sbirs_sensor/config/SbirsRuntimeConfigBuilder.h"
#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsReplaySession.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"
#include "1q/sbirs_sensor/session/SbirsTraceSession.h"
#include "sbirs_sensor/session/SbirsReplayFlatbufferCodec.h"

namespace {

std::string MakeTempTracePath(const char* prefix) {
  static unsigned int unique_counter = 0U;
  const char* temp_dir = std::getenv("TMPDIR");
  if (temp_dir == nullptr || temp_dir[0] == '\0') {
    temp_dir = "/tmp";
  }
  std::ostringstream stream;
  stream << temp_dir;
  const std::string path = stream.str();
  if (!path.empty() && path[path.size() - 1] != '/') {
    stream << "/";
  }
  const long long ticks =
      static_cast<long long>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
  stream << prefix << "-" << std::time(nullptr) << "-" << ticks << "-" << std::rand() << "-"
         << unique_counter++ << ".trace";
  return stream.str();
}

sbirs_sensor::session::SbirsVector3M Vector(double x, double y, double z) {
  sbirs_sensor::session::SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

sbirs_sensor::config::SbirsSessionConfig Config() {
  sbirs_sensor::config::SbirsSessionConfig config;
  config.hardware.noise_equivalent_power_w = 1.0e-18f;
  config.mission.scan_start_az_deg = -1.0f;
  config.mission.scan_end_az_deg = 10.0f;
  config.mission.scan_rate_deg_per_sec = 1.0f;
  config.policy.detection.wide_min_snr_linear = 0.001f;
  config.policy.detection.narrow_min_snr_linear = 0.001f;
  return config;
}

sbirs_sensor::session::SbirsCycleInput ValidInput(std::uint32_t cycle_index,
                                                  double target_offset_y_m = 0.0) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.target_name = "hot";
  target.position_ecef_m = Vector(8000000.0, target_offset_y_m, 0.0);
  target.temperature_k = 2200.0f;
  target.projected_area_m2 = 5000.0f;
  return sbirs_sensor::session::SbirsCycleInputBuilder()
      .WithCycleIndex(cycle_index)
      .WithDeltaTimeSec(1.0f)
      .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
      .AddTarget(target)
      .Build();
}

sbirs_sensor::session::SbirsCycleInput ImmMultiTargetInput(std::uint32_t cycle_index) {
  sbirs_sensor::session::SbirsSceneTarget first;
  first.target_id = 1U;
  first.target_name = "first";
  first.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  first.temperature_k = 2200.0f;
  first.projected_area_m2 = 5000.0f;
  sbirs_sensor::session::SbirsSceneTarget second = first;
  second.target_id = 2U;
  second.target_name = "second";
  second.position_ecef_m = Vector(8000000.0, 5000.0, 0.0);
  return sbirs_sensor::session::SbirsCycleInputBuilder()
      .WithCycleIndex(cycle_index)
      .WithDeltaTimeSec(1.0f)
      .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
      .AddTarget(first)
      .AddTarget(second)
      .Build();
}

sbirs_sensor::session::SbirsCycleInput MovingCueInput(std::uint32_t cycle_index,
                                                      double target_offset_y_m) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 31U;
  target.target_name = "moving-cue";
  target.position_ecef_m = Vector(8000000.0, target_offset_y_m, 0.0);
  target.velocity_ecef_m_per_s = Vector(0.0, 20000.0, 0.0);
  target.has_velocity_ecef_m_per_s = true;
  target.temperature_k = 2200.0f;
  target.projected_area_m2 = 5000.0f;
  return sbirs_sensor::session::SbirsCycleInputBuilder()
      .WithCycleIndex(cycle_index)
      .WithDeltaTimeSec(1.0f)
      .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
      .AddTarget(target)
      .Build();
}

sbirs_sensor::session::SbirsCycleInput PointingInput(std::uint32_t cycle_index,
                                                     double first_offset_y_m,
                                                     bool include_second = false) {
  sbirs_sensor::session::SbirsSceneTarget first;
  first.target_id = 1U;
  first.target_name = "first-pointing";
  first.position_ecef_m = Vector(8000000.0, first_offset_y_m, 0.0);
  first.temperature_k = 2200.0f;
  first.projected_area_m2 = 5000.0f;
  sbirs_sensor::session::SbirsCycleInputBuilder builder;
  builder.WithCycleIndex(cycle_index)
      .WithDeltaTimeSec(1.0f)
      .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
      .AddTarget(first);
  if (include_second) {
    sbirs_sensor::session::SbirsSceneTarget second = first;
    second.target_id = 2U;
    second.target_name = "second-pointing";
    second.position_ecef_m.y = -first_offset_y_m;
    builder.AddTarget(second);
  }
  return builder.Build();
}

const sbirs_sensor::attribution::SbirsDetectionAttributionRecord* FindAttribution(
    const sbirs_sensor::session::SbirsCycleResult& result, std::uint64_t target_id) {
  for (const sbirs_sensor::attribution::SbirsDetectionAttributionRecord& attribution :
       result.detection_attributions) {
    if (attribution.target_id == target_id) {
      return &attribution;
    }
  }
  return nullptr;
}

oneq::replay::ReplayTraceManifest Manifest(const char* trace_id) {
  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = trace_id;
  manifest.module = "sbirs_sensor";
  manifest.scenario_id = "unit-test";
  return manifest;
}

}  // namespace

TEST(SbirsReplaySessionTest, ReplaySbirsTraceRoundtrip) {
  const std::string trace_dir = MakeTempTracePath("oneq-sbirs-replay-roundtrip");
  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, Manifest("sbirs-roundtrip"), true));
    sbirs_sensor::session::SbirsTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;
    sbirs_sensor::session::SbirsTraceSession session(Config(), options);

    const sbirs_sensor::config::SbirsRuntimeConfigPatch patch =
        sbirs_sensor::config::SbirsRuntimeConfigBuilder().WithScanRateDegPerSec(1.0f).Build();
    session.ApplyRuntimeConfig(patch);
    const sbirs_sensor::session::SbirsCycleResult result = session.StepWithResult(ValidInput(1U));
    EXPECT_TRUE(result.executed_this_cycle);
    ASSERT_EQ(result.output_frame.detections.size(), 1U);
    ASSERT_EQ(result.detection_attributions.size(), 1U);
    EXPECT_EQ(result.output_frame.detections.front().observation_stage,
              sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);
    EXPECT_NE(result.output_frame.detections.front().azimuth_deg, 0.0f);
    EXPECT_EQ(result.detection_attributions.front().target_id, 1U);
    EXPECT_FALSE(result.detection_attributions.front().used_truth_assist);
    replay_writer->Flush();
  }

  const sbirs_sensor::session::SbirsReplaySessionResult replay_result =
      sbirs_sensor::session::ReplaySbirsTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_TRUE(replay_result.report.replay_ready);
  EXPECT_EQ(replay_result.playback.applied_input_count, 1U);
  EXPECT_EQ(replay_result.playback.applied_runtime_patch_count, 1U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 1U);
  EXPECT_FALSE(replay_result.playback.divergence_found);
}

TEST(SbirsReplaySessionTest, ReplayPreservesNisLossAndReacquisitionDiagnostics) {
  sbirs_sensor::config::SbirsSessionConfig config = Config();
  config.policy.tracking.nis_gate_loss_cycles = 1U;

  const std::string trace_dir = MakeTempTracePath("oneq-sbirs-replay-nis-loss");
  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, Manifest("sbirs-nis-loss"), true));
    sbirs_sensor::session::SbirsTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;
    sbirs_sensor::session::SbirsTraceSession session(config, options);

    const sbirs_sensor::session::SbirsCycleResult acquired = session.StepWithResult(ValidInput(1U));
    ASSERT_FALSE(acquired.output_frame.detections.empty());
    EXPECT_EQ(acquired.output_frame.detections.front().observation_stage,
              sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);

    // 默认 NFOV 半宽 1°；保持目标在窗口内，使 trace 只覆盖量测后的 NIS 丢锁。
    const sbirs_sensor::session::SbirsCycleResult lost =
        session.StepWithResult(ValidInput(2U, 10000.0));
    const sbirs_sensor::attribution::SbirsDetectionAttributionRecord* lost_attr =
        FindAttribution(lost, 1U);
    ASSERT_NE(lost_attr, nullptr);
    EXPECT_TRUE(lost_attr->has_estimation_nis);
    EXPECT_TRUE(lost_attr->estimation_nis_gate_exceeded);
    EXPECT_EQ(lost_attr->capture_failure_reason,
              sbirs_sensor::attribution::SbirsCaptureFailureReason::kEstimationNisGateLost);
    EXPECT_TRUE(lost.output_frame.detections.empty());

    const sbirs_sensor::session::SbirsCycleResult reacquired =
        session.StepWithResult(ValidInput(3U));
    ASSERT_FALSE(reacquired.output_frame.detections.empty());
    EXPECT_EQ(reacquired.output_frame.detections.front().observation_stage,
              sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);
    replay_writer->Flush();
  }

  const sbirs_sensor::session::SbirsReplaySessionResult replay_result =
      sbirs_sensor::session::ReplaySbirsTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_TRUE(replay_result.report.replay_ready);
  EXPECT_EQ(replay_result.playback.applied_input_count, 3U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 3U);
  EXPECT_FALSE(replay_result.playback.divergence_found);
}

TEST(SbirsReplaySessionTest, ReplayPreservesMultiTargetImmTracking) {
  sbirs_sensor::config::SbirsSessionConfig config = Config();
  config.policy.scheduler.max_concurrent_nfov_locks = 2;
  config.policy.tracking.enable_imm_tracking = true;
  config.policy.tracking.imm_model_noise_diff_coeffs = {0.5f, 80.0f};
  config.policy.error_model.attitude_sigma_deg = 0.0f;
  config.policy.error_model.orbit_sigma_deg = 0.0f;
  config.policy.error_model.fov_sigma_deg = 0.0f;
  config.policy.error_model.range_fraction_sigma = 0.0f;

  const std::string trace_dir = MakeTempTracePath("oneq-sbirs-replay-multi-imm");
  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, Manifest("sbirs-multi-imm"), true));
    sbirs_sensor::session::SbirsTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;
    sbirs_sensor::session::SbirsTraceSession session(config, options);
    const sbirs_sensor::session::SbirsCycleResult acquired =
        session.StepWithResult(ImmMultiTargetInput(1U));
    ASSERT_EQ(acquired.output_frame.detections.size(), 2U);
    const sbirs_sensor::session::SbirsCycleResult tracked =
        session.StepWithResult(ImmMultiTargetInput(2U));
    ASSERT_EQ(tracked.output_frame.detections.size(), 2U);
    replay_writer->Flush();
  }

  const sbirs_sensor::session::SbirsReplaySessionResult replay_result =
      sbirs_sensor::session::ReplaySbirsTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_EQ(replay_result.playback.applied_input_count, 2U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 2U);
  EXPECT_FALSE(replay_result.playback.divergence_found);
}

TEST(SbirsReplaySessionTest, ReplayPreservesMeasurementDerivedCvCue) {
  sbirs_sensor::config::SbirsSessionConfig config = Config();
  config.mission.narrow_cue_latency_s = 1.0f;
  config.mission.narrow_field_fov_az_deg = 1.0f;
  config.policy.error_model.attitude_sigma_deg = 0.0f;
  config.policy.error_model.orbit_sigma_deg = 0.0f;
  config.policy.error_model.fov_sigma_deg = 0.0f;
  config.policy.error_model.range_fraction_sigma = 0.0f;

  const std::string trace_dir = MakeTempTracePath("oneq-sbirs-replay-cv-cue");
  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, Manifest("sbirs-cv-cue"), true));
    sbirs_sensor::session::SbirsTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;
    sbirs_sensor::session::SbirsTraceSession session(config, options);
    const auto first = session.StepWithResult(MovingCueInput(1U, 0.0));
    for (const auto& detection : first.output_frame.detections) {
      EXPECT_NE(detection.observation_stage,
                sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);
    }
    const auto acquired = session.StepWithResult(MovingCueInput(2U, 20000.0));
    ASSERT_EQ(acquired.output_frame.detections.size(), 1U);
    EXPECT_EQ(acquired.output_frame.detections.front().observation_stage,
              sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);
    replay_writer->Flush();
  }

  const sbirs_sensor::session::SbirsReplaySessionResult replay_result =
      sbirs_sensor::session::ReplaySbirsTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_EQ(replay_result.playback.applied_input_count, 2U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 2U);
  EXPECT_FALSE(replay_result.playback.divergence_found);
}

TEST(SbirsReplaySessionTest, ReplayPreservesMultiCycleSlewAndRuntimeMissionPatch) {
  sbirs_sensor::config::SbirsSessionConfig config = Config();
  config.mission.scan_start_az_deg = -10.0f;
  config.mission.scan_end_az_deg = 10.0f;
  config.mission.scan_rate_deg_per_sec = 0.0f;
  config.mission.wide_field_fov_az_deg = 30.0f;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 2.0f;
  const std::string trace_dir = MakeTempTracePath("oneq-sbirs-replay-atp-patch");
  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, Manifest("sbirs-atp-patch"), true));
    sbirs_sensor::session::SbirsTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;
    sbirs_sensor::session::SbirsTraceSession session(config, options);
    const sbirs_sensor::session::SbirsCycleResult slewing =
        session.StepWithResult(PointingInput(1U, 0.0));
    ASSERT_EQ(slewing.output_frame.detections.size(), 1U);
    EXPECT_EQ(slewing.output_frame.detections.front().observation_stage,
              sbirs_sensor::output::SbirsObservationStage::kWideFieldSearch);
    EXPECT_EQ(slewing.detection_attributions.front().nfov_channel_id, 0);

    config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 20.0f;
    session.ApplyRuntimeConfig(
        sbirs_sensor::config::SbirsRuntimeConfigBuilder().WithMission(config.mission).Build());
    const sbirs_sensor::session::SbirsCycleResult acquired =
        session.StepWithResult(PointingInput(2U, 0.0));
    ASSERT_EQ(acquired.output_frame.detections.size(), 1U);
    EXPECT_EQ(acquired.output_frame.detections.front().observation_stage,
              sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);
    replay_writer->Flush();
  }

  const sbirs_sensor::session::SbirsReplaySessionResult replay_result =
      sbirs_sensor::session::ReplaySbirsTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_EQ(replay_result.playback.applied_input_count, 2U);
  EXPECT_EQ(replay_result.playback.applied_runtime_patch_count, 1U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 2U);
  EXPECT_FALSE(replay_result.playback.divergence_found);
}

TEST(SbirsReplaySessionTest, ReplayPreservesDualChannelPointingTimeout) {
  sbirs_sensor::config::SbirsSessionConfig config = Config();
  config.mission.scan_start_az_deg = 0.0f;
  config.mission.scan_end_az_deg = 20.0f;
  config.mission.scan_rate_deg_per_sec = 0.0f;
  config.mission.wide_field_fov_az_deg = 30.0f;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 1.0f;
  config.policy.scheduler.max_concurrent_nfov_locks = 2;
  const std::string trace_dir = MakeTempTracePath("oneq-sbirs-replay-atp-timeout");
  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, Manifest("sbirs-atp-timeout"), true));
    sbirs_sensor::session::SbirsTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;
    sbirs_sensor::session::SbirsTraceSession session(config, options);
    sbirs_sensor::session::SbirsCycleResult result;
    for (std::uint32_t cycle = 1U; cycle <= 180U; ++cycle) {
      const double offset_y_m = cycle % 2U == 0U ? -176326.9807 : 176326.9807;
      result = session.StepWithResult(PointingInput(cycle, offset_y_m, true));
    }
    EXPECT_TRUE(result.output_frame.detections.empty());
    ASSERT_EQ(result.detection_attributions.size(), 2U);
    for (const sbirs_sensor::attribution::SbirsDetectionAttributionRecord& attribution :
         result.detection_attributions) {
      EXPECT_GE(attribution.nfov_channel_id, 0);
      EXPECT_EQ(attribution.capture_failure_reason,
                sbirs_sensor::attribution::SbirsCaptureFailureReason::kNfovPointingTimeout);
    }
    replay_writer->Flush();
  }

  const sbirs_sensor::session::SbirsReplaySessionResult replay_result =
      sbirs_sensor::session::ReplaySbirsTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_EQ(replay_result.playback.applied_input_count, 180U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 180U);
  EXPECT_FALSE(replay_result.playback.divergence_found);
}

TEST(SbirsReplaySessionTest, ReplayPreservesTrackingCoastAndGateLoss) {
  sbirs_sensor::config::SbirsSessionConfig config = Config();
  config.mission.narrow_field_fov_az_deg = 1.0f;
  config.mission.narrow_pointing_max_slew_rate_deg_per_sec = 0.1f;
  const std::string trace_dir = MakeTempTracePath("oneq-sbirs-replay-tracking-gate");
  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, Manifest("sbirs-tracking-gate"), true));
    sbirs_sensor::session::SbirsTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;
    sbirs_sensor::session::SbirsTraceSession session(config, options);
    session.StepWithResult(ValidInput(1U));
    const auto coast = session.StepWithResult(ValidInput(2U, 35000.0));
    EXPECT_TRUE(coast.output_frame.detections.empty());
    ASSERT_EQ(coast.detection_attributions.size(), 1U);
    EXPECT_TRUE(coast.detection_attributions[0].nfov_tracking_coasting);
    EXPECT_EQ(coast.detection_attributions[0].nfov_tracking_gate_failure_count, 1U);
    const auto lost = session.StepWithResult(ValidInput(3U, 35000.0));
    EXPECT_TRUE(lost.output_frame.detections.empty());
    ASSERT_EQ(lost.detection_attributions.size(), 1U);
    EXPECT_EQ(lost.detection_attributions[0].capture_failure_reason,
              sbirs_sensor::attribution::SbirsCaptureFailureReason::kNfovTrackingGateLost);
    replay_writer->Flush();
  }

  const sbirs_sensor::session::SbirsReplaySessionResult replay_result =
      sbirs_sensor::session::ReplaySbirsTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_EQ(replay_result.playback.applied_input_count, 3U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 3U);
  EXPECT_FALSE(replay_result.playback.divergence_found);
}

TEST(SbirsReplaySessionTest, ReplayPreservesPointingDisturbanceAndRuntimePolicyPatch) {
  sbirs_sensor::config::SbirsSessionConfig config = Config();
  config.mission.narrow_field_fov_az_deg = 10.0f;
  config.mission.narrow_field_fov_el_deg = 10.0f;
  config.policy.scheduler.max_concurrent_nfov_locks = 2;
  config.policy.pointing_disturbance.common_attitude_sigma_deg = 0.05f;
  config.policy.pointing_disturbance.common_attitude_correlation_time_s = 2.0f;
  config.policy.pointing_disturbance.channel_pointing_sigma_deg = 0.1f;
  config.policy.pointing_disturbance.channel_pointing_correlation_time_s = 3.0f;
  config.policy.pointing_disturbance.channel_vibration_amplitude_deg = 0.2f;
  config.policy.pointing_disturbance.channel_vibration_frequency_hz = 0.25f;
  config.policy.pointing_disturbance.random_seed = 61U;
  const std::string trace_dir = MakeTempTracePath("oneq-sbirs-replay-pointing-disturbance");
  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, Manifest("sbirs-pointing-disturbance"),
                                            true));
    sbirs_sensor::session::SbirsTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;
    sbirs_sensor::session::SbirsTraceSession session(config, options);
    session.StepWithResult(ImmMultiTargetInput(1U));
    const auto tracked = session.StepWithResult(ImmMultiTargetInput(2U));
    ASSERT_EQ(tracked.detection_attributions.size(), 2U);
    EXPECT_GT(tracked.detection_attributions[0].nfov_pointing_error_deg, 0.0f);

    config.policy.pointing_disturbance.random_seed = 67U;
    config.policy.pointing_disturbance.channel_vibration_amplitude_deg = 0.1f;
    session.ApplyRuntimeConfig(
        sbirs_sensor::config::SbirsRuntimeConfigBuilder().WithPolicy(config.policy).Build());
    session.StepWithResult(ImmMultiTargetInput(3U));
    session.StepWithResult(ImmMultiTargetInput(4U));
    replay_writer->Flush();
  }

  const sbirs_sensor::session::SbirsReplaySessionResult replay_result =
      sbirs_sensor::session::ReplaySbirsTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_EQ(replay_result.playback.applied_input_count, 4U);
  EXPECT_EQ(replay_result.playback.applied_runtime_patch_count, 1U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 4U);
  EXPECT_FALSE(replay_result.playback.divergence_found);
}

TEST(SbirsReplaySessionTest, DisturbanceConfigMismatchTriggersDivergence) {
  sbirs_sensor::config::SbirsSessionConfig replay_config = Config();
  replay_config.mission.scan_start_az_deg = 0.0f;
  replay_config.mission.scan_rate_deg_per_sec = 0.0f;
  replay_config.mission.wide_field_fov_az_deg = 0.02f;
  replay_config.mission.wide_field_fov_el_deg = 0.02f;
  replay_config.policy.pointing_disturbance.common_attitude_sigma_deg = 5.0f;
  replay_config.policy.pointing_disturbance.random_seed = 53U;
  sbirs_sensor::config::SbirsSessionConfig oracle_config = replay_config;
  oracle_config.policy.pointing_disturbance.common_attitude_sigma_deg = 0.0f;

  const std::string trace_dir = MakeTempTracePath("oneq-sbirs-replay-disturbance-divergence");
  oneq::replay::ReplayTraceWriter writer(trace_dir, Manifest("sbirs-disturbance-divergence"), true);
  oneq::replay::ReplayTraceEvent config_event;
  config_event.module = "sbirs_sensor";
  config_event.event_type = "session_config";
  config_event.payload_type = "SbirsSessionConfig";
  config_event.payload_encoding = "flatbuffers";
  config_event.payload_bytes = sbirs_sensor::session::EncodeSbirsSessionConfig(replay_config);
  writer.WriteEvent(config_event);

  const sbirs_sensor::session::SbirsCycleInput input = ValidInput(1U);
  oneq::replay::ReplayTraceEvent input_event;
  input_event.module = "sbirs_sensor";
  input_event.event_type = "cycle_input";
  input_event.payload_type = "SbirsCycleInput";
  input_event.payload_encoding = "flatbuffers";
  input_event.payload_bytes = sbirs_sensor::session::EncodeSbirsCycleInput(input);
  input_event.has_cycle_index = true;
  input_event.cycle_index = input.cycle_index;
  writer.WriteEvent(input_event);

  sbirs_sensor::session::SbirsSession oracle =
      sbirs_sensor::session::SbirsSession::Create(oracle_config);
  oneq::replay::ReplayTraceEvent output_event;
  output_event.module = "sbirs_sensor";
  output_event.event_type = "cycle_output";
  output_event.payload_type = "SbirsCycleResult";
  output_event.payload_encoding = "flatbuffers";
  output_event.payload_bytes =
      sbirs_sensor::session::EncodeSbirsCycleResult(oracle.StepWithResult(input));
  output_event.has_cycle_index = true;
  output_event.cycle_index = input.cycle_index;
  writer.WriteEvent(output_event);
  writer.Flush();

  const sbirs_sensor::session::SbirsReplaySessionResult replay_result =
      sbirs_sensor::session::ReplaySbirsTrace(trace_dir);
  EXPECT_FALSE(replay_result.ok);
  EXPECT_TRUE(replay_result.playback.divergence_found);
}

TEST(SbirsReplaySessionTest, ReplaySbirsTraceRejectsWrongModule) {
  const std::string trace_dir = MakeTempTracePath("oneq-sbirs-replay-wrong-module");
  oneq::replay::ReplayTraceManifest manifest = Manifest("sbirs-wrong-module");
  manifest.module = "electro_optical_sensor";
  oneq::replay::ReplayTraceWriter writer(trace_dir, manifest, true);
  writer.Flush();

  const sbirs_sensor::session::SbirsReplaySessionResult replay_result =
      sbirs_sensor::session::ReplaySbirsTrace(trace_dir);
  EXPECT_FALSE(replay_result.ok);
  EXPECT_FALSE(replay_result.report.replay_ready);
}

TEST(SbirsReplaySessionTest, ReplaySbirsTraceDetectsDivergence) {
  const std::string trace_dir = MakeTempTracePath("oneq-sbirs-replay-divergence");
  oneq::replay::ReplayTraceWriter writer(trace_dir, Manifest("sbirs-divergence"), true);

  oneq::replay::ReplayTraceEvent config_event;
  config_event.module = "sbirs_sensor";
  config_event.event_type = "session_config";
  config_event.payload_type = "SbirsSessionConfig";
  config_event.payload_encoding = "flatbuffers";
  config_event.payload_bytes = sbirs_sensor::session::EncodeSbirsSessionConfig(Config());
  writer.WriteEvent(config_event);

  const sbirs_sensor::session::SbirsCycleInput input = ValidInput(1U);
  oneq::replay::ReplayTraceEvent input_event;
  input_event.module = "sbirs_sensor";
  input_event.event_type = "cycle_input";
  input_event.payload_type = "SbirsCycleInput";
  input_event.payload_encoding = "flatbuffers";
  input_event.payload_bytes = sbirs_sensor::session::EncodeSbirsCycleInput(input);
  input_event.has_cycle_index = true;
  input_event.cycle_index = input.cycle_index;
  writer.WriteEvent(input_event);

  sbirs_sensor::session::SbirsSession oracle =
      sbirs_sensor::session::SbirsSession::Create(Config());
  sbirs_sensor::session::SbirsCycleResult tampered = oracle.StepWithResult(input);
  ASSERT_EQ(tampered.detection_attributions.size(), 1U);
  tampered.detection_attributions.front().has_nfov_tracking_diagnostics = true;
  tampered.detection_attributions.front().nfov_pointing_error_deg = 99.0f;
  oneq::replay::ReplayTraceEvent output_event;
  output_event.module = "sbirs_sensor";
  output_event.event_type = "cycle_output";
  output_event.payload_type = "SbirsCycleResult";
  output_event.payload_encoding = "flatbuffers";
  output_event.payload_bytes = sbirs_sensor::session::EncodeSbirsCycleResult(tampered);
  output_event.has_cycle_index = true;
  output_event.cycle_index = 1U;
  writer.WriteEvent(output_event);
  writer.Flush();

  const sbirs_sensor::session::SbirsReplaySessionResult replay_result =
      sbirs_sensor::session::ReplaySbirsTrace(trace_dir);
  EXPECT_FALSE(replay_result.ok);
  EXPECT_TRUE(replay_result.playback.divergence_found);
}
