/**
 * @file sar_replay_session_test.cpp
 * @brief 验证 SAR RecordingSession 与 ReplaySession 的 replay 闭环。
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <sstream>
#include <string>

#include "1q/replay/ReplayTrace.h"
#include "1q/sar/session/SarReplaySession.h"
#include "1q/sar/session/SarSession.h"
#include "1q/sar/session/SarRecordingSession.h"
#include "sar/session/SarReplayFlatbufferCodec.h"

namespace {

/**
 * @brief RAII 临时目录包装，在析构时删除目录及其内容。
 *
 * 每个实例创建后生成一个唯一目录路径，使用 testing::TempDir() 作为基目录。
 * 析构时删除整个目录，确保测试失败时也不会遗留临时文件。
 */
class ScopedTempDir {
 public:
  ScopedTempDir() {
    const std::string base = testing::TempDir();
    const long long ticks =
        static_cast<long long>(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    std::ostringstream stream;
    stream << base << "sar-replay-" << std::time(nullptr) << "-" << ticks << "-"
           << std::rand();
    path_ = stream.str();
  }

  ~ScopedTempDir() {
    if (!path_.empty()) {
      RemoveAll(path_);
    }
  }

  ScopedTempDir(const ScopedTempDir&) = delete;
  ScopedTempDir& operator=(const ScopedTempDir&) = delete;

  const std::string& Path() const { return path_; }

 private:
  static void RemoveAll(const std::string& dir) {
    const std::string cmd = "rm -rf " + dir;
    std::system(cmd.c_str());
  }

  std::string path_;
};

}  // namespace

namespace sar {
namespace session {
namespace tests {

config::SarSessionConfig MakeSmallRdaConfigForReplay() {
  config::SarSessionConfig config;
  config.hardware.carrier_frequency_hz = 1.0e9;
  config.hardware.bandwidth_hz = 25.0e6;
  config.hardware.pulse_width_s = 0.16e-6;
  config.hardware.pulse_repetition_frequency_hz = 20.0;
  config.hardware.sample_rate_hz = 100.0e6;
  config.mission.nominal_slant_range_m = 29.9792458;
  config.mission.scene_center_latitude_deg =
      29.9792458 / 6378137.0 * 180.0 / 3.14159265358979323846;
  config.mission.platform_speed_mps = 2.0;
  config.mission.range_sample_count = 64U;
  config.mission.azimuth_pulse_count = 9U;
  config.policy.enable_l1_rda_imaging = true;
  config.policy.max_allowed_squint_angle_deg = 89.0;
  return config;
}

config::SarSessionConfig MakeSmallL3BpConfigForReplay() {
  config::SarSessionConfig config = MakeSmallRdaConfigForReplay();
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

SarCycleInput MakeReplayInput(std::uint32_t cycle_index = 1U) {
  SarCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 0.1f;
  input.platform.latitude_deg = 0.0;
  input.platform.longitude_deg = 0.0;
  input.platform.altitude_m = 0.0;
  input.platform.velocity_east_mps = 2.0;

  SarPointTarget target;
  target.latitude_deg = 29.9792458 / 6378137.0 * 180.0 / 3.14159265358979323846;
  target.longitude_deg = 0.0;
  target.altitude_m = 0.0;
  target.radar_cross_section_dbsm = 80.0;
  input.point_targets.push_back(target);
  return input;
}

SarCycleInput MakeExternalRawIqReplayInput(std::uint32_t cycle_index = 1U) {
  SarCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 0.1f;
  input.raw_iq.samples_per_pulse = 64U;
  input.raw_iq.i_values.assign(9U * 64U, 0.0);
  input.raw_iq.q_values.assign(9U * 64U, 0.0);
  for (std::size_t row = 0U; row < 9U; ++row) {
    input.raw_iq.i_values[row * 64U + 20U] = 1.0;
  }
  return input;
}

TEST(SarReplaySessionTest, ReplaySarTraceRoundtrip) {
  ScopedTempDir temp_dir;
  const std::string& trace_dir = temp_dir.Path();

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "sar-replay-roundtrip-test";
  manifest.module = "sar";
  manifest.scenario_id = "unit-test";

  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));
    SarRecordingSessionOptions options;
    options.replay_writer = replay_writer;
    options.record_config_on_construct = true;
    config::SarSessionConfig config = MakeSmallRdaConfigForReplay();
    config.policy.enable_l2_motion_compensation = true;
    config.mission.l2_velocity_error_stddev_y_mps = 30.0;
    config.mission.l2_velocity_error_stddev_z_mps = 10.0;
    config.mission.l2_random_seed = 2026U;
    SarRecordingSession session(config, options);

    const SarCycleResult result = session.StepWithResult(MakeReplayInput());
    ASSERT_EQ(result.status, SarCycleStatus::kCompleted);
    ASSERT_TRUE(result.product.output_frame.has_l1_image);
    ASSERT_GE(result.issues.size(), 3U);
    replay_writer->Flush();
  }

  const SarReplaySessionResult replay_result = ReplaySarTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_TRUE(replay_result.report.replay_ready);
  EXPECT_EQ(replay_result.playback.applied_input_count, 1U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 1U);
  EXPECT_FALSE(replay_result.playback.divergence_found);
  EXPECT_FALSE(replay_result.reached_failure_marker);
}

TEST(SarReplaySessionTest, ReplayExternalRawIqTraceRoundtrip) {
  ScopedTempDir temp_dir;
  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "sar-external-raw-iq-replay-roundtrip";
  manifest.module = "sar";
  manifest.scenario_id = "unit-test";

  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(temp_dir.Path(), manifest, true));
    SarRecordingSessionOptions options;
    options.replay_writer = replay_writer;
    options.record_config_on_construct = true;
    config::SarSessionConfig config = MakeSmallRdaConfigForReplay();
    SarRecordingSession session(config, options);

    // 外部 IQ 输入仍无条件记录进 cycle_input（可回放）；IQ 不再进周期记录（规则 15e）。
    const SarCycleResult result = session.StepWithResult(MakeExternalRawIqReplayInput());
    ASSERT_EQ(result.status, SarCycleStatus::kCompleted) << static_cast<int>(result.abort_reason);
    ASSERT_TRUE(result.product.output_frame.has_l1_image);
    replay_writer->Flush();
  }

  const SarReplaySessionResult replay_result = ReplaySarTrace(temp_dir.Path());
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_EQ(replay_result.playback.applied_input_count, 1U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 1U);
  EXPECT_FALSE(replay_result.playback.divergence_found);
}

TEST(SarReplaySessionTest, ReplayExternalRawIqShapeFailureRoundtrip) {
  ScopedTempDir temp_dir;
  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "sar-external-raw-iq-failure-roundtrip";
  manifest.module = "sar";
  manifest.scenario_id = "unit-test";

  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(temp_dir.Path(), manifest, true));
    SarRecordingSessionOptions options;
    options.replay_writer = replay_writer;
    options.record_config_on_construct = true;
    SarRecordingSession session(MakeSmallRdaConfigForReplay(), options);
    SarCycleInput malformed = MakeExternalRawIqReplayInput();
    malformed.raw_iq.i_values.pop_back();

    const SarCycleResult result = session.StepWithResult(malformed);
    ASSERT_EQ(result.status, session::SarCycleStatus::kRejectedInvalidInput);
    ASSERT_EQ(result.abort_reason, SarPipelineAbortReason::kExternalInputRejected);
    replay_writer->Flush();
  }

  const SarReplaySessionResult replay_result = ReplaySarTrace(temp_dir.Path());
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_TRUE(replay_result.reached_failure_marker);
  EXPECT_EQ(replay_result.playback.applied_input_count, 1U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 1U);
  EXPECT_FALSE(replay_result.playback.divergence_found);
}

TEST(SarReplaySessionTest, ReplayContinuesAfterFailureMarker) {
  ScopedTempDir temp_dir;
  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "sar-replay-marker-continuation";
  manifest.module = "sar";
  manifest.scenario_id = "unit-test";
  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> writer(
        new oneq::replay::ReplayTraceWriter(temp_dir.Path(), manifest, true));
    SarRecordingSessionOptions options;
    options.replay_writer = writer;
    options.record_config_on_construct = true;
    SarRecordingSession session(MakeSmallRdaConfigForReplay(), options);
    ASSERT_EQ(session.StepWithResult(MakeReplayInput(1U)).status, SarCycleStatus::kCompleted);
    oneq::replay::ReplayTraceFailure failure;
    failure.error_code = "SAR_RECOVERABLE_TEST";
    failure.message = "synthetic recoverable marker";
    failure.has_cycle_index = true;
    failure.cycle_index = 1U;
    writer->WriteFailureMarker(failure);
    ASSERT_EQ(session.StepWithResult(MakeReplayInput(2U)).status, SarCycleStatus::kCompleted);
    writer->Flush();
  }
  const SarReplaySessionResult replay_result = ReplaySarTrace(temp_dir.Path());
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_TRUE(replay_result.reached_failure_marker);
  EXPECT_EQ(replay_result.playback.failure_marker_count, 1U);
  EXPECT_EQ(replay_result.playback.applied_input_count, 2U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 2U);
}

TEST(SarReplaySessionTest, ReplayL3BpTraceRoundtrip) {
  ScopedTempDir temp_dir;
  const std::string& trace_dir = temp_dir.Path();

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "sar-l3-bp-replay-roundtrip-test";
  manifest.module = "sar";
  manifest.scenario_id = "unit-test";

  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));
    SarRecordingSessionOptions options;
    options.replay_writer = replay_writer;
    options.record_config_on_construct = true;
    SarRecordingSession session(MakeSmallL3BpConfigForReplay(), options);

    const SarCycleResult first = session.StepWithResult(MakeReplayInput(1U));
    ASSERT_EQ(first.status, SarCycleStatus::kCompleted);
    ASSERT_TRUE(first.product.output_frame.has_l3_bp_image);
    ASSERT_EQ(first.product.output_frame.completed_stage, SarProcessingStage::kL3BpImage);
    const SarCycleResult second = session.StepWithResult(MakeReplayInput(2U));
    ASSERT_EQ(second.status, SarCycleStatus::kCompleted);
    ASSERT_TRUE(second.product.output_frame.has_l3_bp_image);
    replay_writer->Flush();
  }

  const SarReplaySessionResult replay_result = ReplaySarTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_TRUE(replay_result.report.replay_ready);
  EXPECT_EQ(replay_result.playback.applied_input_count, 2U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 2U);
  EXPECT_FALSE(replay_result.playback.divergence_found);
  EXPECT_FALSE(replay_result.reached_failure_marker);
}

TEST(SarReplaySessionTest, ReplaySarTraceAppliesRuntimePatchBeforeCycle) {
  ScopedTempDir temp_dir;
  const std::string& trace_dir = temp_dir.Path();

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "sar-replay-runtime-patch-test";
  manifest.module = "sar";
  manifest.scenario_id = "unit-test";

  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));
    config::SarSessionConfig config = MakeSmallRdaConfigForReplay();
    config.policy.enable_l1_rda_imaging = false;
    SarRecordingSessionOptions options;
    options.replay_writer = replay_writer;
    options.record_config_on_construct = true;
    SarRecordingSession session(config, options);

    config::SarRuntimeConfigPatch patch;
    patch.has_enable_l1_rda_imaging = true;
    patch.enable_l1_rda_imaging = true;
    (void)session.TryApplyRuntimeConfig(patch);
    const SarCycleResult result = session.StepWithResult(MakeReplayInput(2U));
    ASSERT_TRUE(result.product.output_frame.has_l1_image);
    replay_writer->Flush();
  }

  const SarReplaySessionResult replay_result = ReplaySarTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_EQ(replay_result.playback.applied_runtime_patch_count, 1U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 1U);
}

TEST(SarReplaySessionTest, ReplaySarTraceDetectsFocusedImagePixelDivergence) {
  ScopedTempDir temp_dir;
  const std::string& trace_dir = temp_dir.Path();

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "sar-focused-image-divergence-test";
  manifest.module = "sar";
  manifest.scenario_id = "unit-test";

  const config::SarSessionConfig config = MakeSmallRdaConfigForReplay();
  const SarCycleInput input = MakeReplayInput();
  SarSession session = SarSession::Create(config);
  SarCycleResult expected = session.StepWithResult(input);
  ASSERT_EQ(expected.status, SarCycleStatus::kCompleted);
  ASSERT_FALSE(expected.product.focused_image.real_values.empty());
  expected.product.focused_image.real_values.front() += 1.0;

  oneq::replay::ReplayTraceWriter writer(trace_dir, manifest, true);
  oneq::replay::ReplayTraceEvent config_event;
  config_event.module = "sar";
  config_event.event_type = "session_config";
  config_event.payload_type = "SarSessionConfig";
  config_event.payload_encoding = "flatbuffers";
  config_event.payload_bytes = EncodeSarSessionConfig(config);
  writer.WriteEvent(config_event);

  oneq::replay::ReplayTraceEvent input_event;
  input_event.module = "sar";
  input_event.event_type = "cycle_input";
  input_event.payload_type = "SarCycleInput";
  input_event.payload_encoding = "flatbuffers";
  input_event.payload_bytes = EncodeSarCycleInput(input);
  input_event.has_cycle_index = true;
  input_event.cycle_index = input.cycle_index;
  writer.WriteEvent(input_event);

  oneq::replay::ReplayTraceEvent output_event;
  output_event.module = "sar";
  output_event.event_type = "cycle_output";
  output_event.payload_type = "SarCycleResult";
  output_event.payload_encoding = "flatbuffers";
  output_event.payload_bytes = EncodeSarCycleResult(expected);
  output_event.has_cycle_index = true;
  output_event.cycle_index = input.cycle_index;
  writer.WriteEvent(output_event);
  writer.Flush();

  const SarReplaySessionResult replay_result = ReplaySarTrace(trace_dir);
  EXPECT_FALSE(replay_result.ok);
  EXPECT_TRUE(replay_result.playback.divergence_found);
  EXPECT_NE(replay_result.first_error.find("SarCycleResult"), std::string::npos);
}

TEST(SarReplaySessionTest, ReplaySarTraceRejectsStandaloneOutputFramePayload) {
  ScopedTempDir temp_dir;
  const std::string& trace_dir = temp_dir.Path();

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "sar-standalone-output-frame-rejected-test";
  manifest.module = "sar";
  manifest.scenario_id = "unit-test";

  oneq::replay::ReplayTraceWriter writer(trace_dir, manifest, true);
  oneq::replay::ReplayTraceEvent config_event;
  config_event.module = "sar";
  config_event.event_type = "session_config";
  config_event.payload_type = "SarSessionConfig";
  config_event.payload_encoding = "flatbuffers";
  config_event.payload_bytes = EncodeSarSessionConfig(MakeSmallRdaConfigForReplay());
  writer.WriteEvent(config_event);

  const SarCycleInput input = MakeReplayInput();
  oneq::replay::ReplayTraceEvent input_event;
  input_event.module = "sar";
  input_event.event_type = "cycle_input";
  input_event.payload_type = "SarCycleInput";
  input_event.payload_encoding = "flatbuffers";
  input_event.payload_bytes = EncodeSarCycleInput(input);
  input_event.has_cycle_index = true;
  input_event.cycle_index = input.cycle_index;
  writer.WriteEvent(input_event);

  oneq::replay::ReplayTraceEvent output_event;
  output_event.module = "sar";
  output_event.event_type = "cycle_output";
  output_event.payload_type = "SarOutputFrame";
  output_event.payload_encoding = "flatbuffers";
  output_event.payload_bytes = EncodeSarCycleResult(SarCycleResult{});
  output_event.has_cycle_index = true;
  output_event.cycle_index = input.cycle_index;
  writer.WriteEvent(output_event);
  writer.Flush();

  const SarReplaySessionResult replay_result = ReplaySarTrace(trace_dir);
  EXPECT_FALSE(replay_result.ok);
  EXPECT_NE(replay_result.first_error.find("does not support cycle_output payload type"),
            std::string::npos);
}

TEST(SarReplaySessionTest, ReplaySarTraceRejectsWrongModule) {
  ScopedTempDir temp_dir;
  const std::string& trace_dir = temp_dir.Path();

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "sar-replay-wrong-module-test";
  manifest.module = "wrong_module";
  manifest.scenario_id = "unit-test";

  {
    oneq::replay::ReplayTraceWriter writer(trace_dir, manifest, true);
    writer.Flush();
  }

  const SarReplaySessionResult replay_result = ReplaySarTrace(trace_dir);
  EXPECT_FALSE(replay_result.ok);
}

TEST(SarReplaySessionTest, ReplaySarTraceRejectsTrailingCycleInput) {
  ScopedTempDir temp_dir;
  const std::string& trace_dir = temp_dir.Path();

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "sar-replay-trailing-input-test";
  manifest.module = "sar";
  manifest.scenario_id = "unit-test";

  oneq::replay::ReplayTraceWriter writer(trace_dir, manifest, true);
  oneq::replay::ReplayTraceEvent config_event;
  config_event.module = "sar";
  config_event.event_type = "session_config";
  config_event.payload_type = "SarSessionConfig";
  config_event.payload_encoding = "flatbuffers";
  config_event.payload_bytes = EncodeSarSessionConfig(MakeSmallRdaConfigForReplay());
  writer.WriteEvent(config_event);

  SarCycleInput input = MakeReplayInput(3U);
  oneq::replay::ReplayTraceEvent input_event;
  input_event.module = "sar";
  input_event.event_type = "cycle_input";
  input_event.payload_type = "SarCycleInput";
  input_event.payload_encoding = "flatbuffers";
  input_event.payload_bytes = EncodeSarCycleInput(input);
  input_event.has_cycle_index = true;
  input_event.cycle_index = input.cycle_index;
  writer.WriteEvent(input_event);
  writer.Flush();

  const SarReplaySessionResult replay_result = ReplaySarTrace(trace_dir);
  EXPECT_FALSE(replay_result.ok);
  EXPECT_NE(replay_result.first_error.find("pending cycle_input"), std::string::npos);
}

}  // namespace tests
}  // namespace session
}  // namespace sar
