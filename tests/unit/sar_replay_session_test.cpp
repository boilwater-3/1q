/**
 * @file sar_replay_session_test.cpp
 * @brief 验证 SAR TraceSession 与 ReplaySession 的 replay trace 闭环。
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
#include "1q/sar/session/SarTraceSession.h"
#include "sar/session/SarReplayFlatbufferCodec.h"

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
  config.mission.platform_speed_mps = 2.0;
  config.mission.range_sample_count = 64U;
  config.mission.azimuth_pulse_count = 9U;
  config.policy.enable_l1_rda_imaging = true;
  return config;
}

SarCycleInput MakeReplayInput(std::uint32_t cycle_index = 1U) {
  SarCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 0.1;
  input.platform.latitude_deg = 0.0;
  input.platform.longitude_deg = 0.0;
  input.platform.altitude_m = 0.0;

  SarPointTarget target;
  target.latitude_deg = 29.9792458 / 6378137.0 * 180.0 / 3.14159265358979323846;
  target.longitude_deg = 0.0;
  target.altitude_m = 0.0;
  target.radar_cross_section_dbsm = 80.0;
  input.point_targets.push_back(target);
  return input;
}

TEST(SarReplaySessionTest, ReplaySarTraceRoundtrip) {
  const std::string trace_dir = MakeTempTracePath("oneq-sar-replay-roundtrip");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "sar-replay-roundtrip-test";
  manifest.module = "sar";
  manifest.scenario_id = "unit-test";

  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));
    SarTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;
    SarTraceSession session(MakeSmallRdaConfigForReplay(), options);

    const SarCycleResult result = session.StepWithResult(MakeReplayInput());
    ASSERT_TRUE(result.executed_this_cycle);
    ASSERT_TRUE(result.output_frame.has_l1_image);
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

TEST(SarReplaySessionTest, ReplaySarTraceAppliesRuntimePatchBeforeCycle) {
  const std::string trace_dir = MakeTempTracePath("oneq-sar-replay-runtime-patch");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "sar-replay-runtime-patch-test";
  manifest.module = "sar";
  manifest.scenario_id = "unit-test";

  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));
    config::SarSessionConfig config = MakeSmallRdaConfigForReplay();
    config.policy.enable_l1_rda_imaging = false;
    SarTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;
    SarTraceSession session(config, options);

    config::SarRuntimeConfigPatch patch;
    patch.has_enable_l1_rda_imaging = true;
    patch.enable_l1_rda_imaging = true;
    session.ApplyRuntimeConfig(patch);
    const SarCycleResult result = session.StepWithResult(MakeReplayInput(2U));
    ASSERT_TRUE(result.output_frame.has_l1_image);
    replay_writer->Flush();
  }

  const SarReplaySessionResult replay_result = ReplaySarTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_EQ(replay_result.playback.applied_runtime_patch_count, 1U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 1U);
}

TEST(SarReplaySessionTest, ReplaySarTraceRejectsWrongModule) {
  const std::string trace_dir = MakeTempTracePath("oneq-sar-replay-wrong-module");

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
  const std::string trace_dir = MakeTempTracePath("oneq-sar-replay-trailing-input");

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
