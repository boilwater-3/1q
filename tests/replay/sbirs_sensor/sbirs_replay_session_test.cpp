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
  config.policy.error_model.angular_sigma_deg = 0.0f;
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

    const sbirs_sensor::session::SbirsCycleResult acquired =
        session.StepWithResult(ValidInput(1U));
    ASSERT_FALSE(acquired.output_frame.detections.empty());
    EXPECT_EQ(acquired.output_frame.detections.front().observation_stage,
              sbirs_sensor::output::SbirsObservationStage::kNarrowFieldAcquisition);

    const sbirs_sensor::session::SbirsCycleResult lost =
        session.StepWithResult(ValidInput(2U, 500000.0));
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

  sbirs_sensor::session::SbirsCycleResult tampered;
  tampered.input_cycle_index = 1U;
  tampered.output_frame.cycle_index = 1U;
  tampered.output_frame.scan_azimuth_deg = 777.0f;
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
