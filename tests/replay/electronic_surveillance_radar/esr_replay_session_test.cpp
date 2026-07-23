/**
 * @file esr_replay_session_test.cpp
 * @brief 验证 ESR ReplaySession 能够回放 trace 并比对输出。
 */

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <sstream>
#include <string>

#include "1q/electronic_surveillance_radar/config/EsrSessionConfig.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInputAdapter.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrReplaySession.h"
#include "1q/electronic_surveillance_radar/session/EsrTraceSession.h"
#include "1q/replay/ReplayTrace.h"
#include "electronic_surveillance_radar/session/EsrReplayFlatbufferCodec.h"

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

namespace electronic_surveillance_radar {
namespace session {
namespace tests {

TEST(EsrReplaySessionTest, ReplayEsrTraceRoundtrip) {
  const std::string trace_dir = MakeTempTracePath("oneq-esr-replay-roundtrip");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "esr-replay-roundtrip-test";
  manifest.module = "electronic_surveillance_radar";
  manifest.scenario_id = "unit-test";

  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));

    config::EsrSessionConfig config;
    config.hardware.beam_az_width_deg = 120.0f;
    config.hardware.beam_el_width_deg = 40.0f;

    EsrTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;
    EsrTraceSession session(config, options);

    EsrCycleInput input;
    input.cycle_index = 1U;
    input.dt_sec = 1.0f;

    const EsrCycleResult result = session.StepWithResult(input);
    EXPECT_GE(result.output_frame.observation_output.observations.size(), 0U);
    replay_writer->Flush();
  }

  const EsrReplaySessionResult replay_result = ReplayEsrTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_TRUE(replay_result.report.replay_ready);
  EXPECT_EQ(replay_result.playback.applied_input_count, 1U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 1U);
  EXPECT_FALSE(replay_result.playback.divergence_found);
  EXPECT_FALSE(replay_result.reached_failure_marker);
}

TEST(EsrReplaySessionTest, ReplayPreservesRealTenKilometerGeometry) {
  const std::string trace_dir = MakeTempTracePath("oneq-esr-replay-10km-geometry");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "esr-replay-10km-geometry-test";
  manifest.module = "electronic_surveillance_radar";
  manifest.scenario_id = "real-10km-geometry";

  config::EsrSessionConfig config;
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 40.0f;

  EsrExternalPoseInput platform;
  platform.platform_position_ecef_m.x_m = -2289512.0;
  platform.platform_position_ecef_m.y_m = 4909946.0;
  platform.platform_position_ecef_m.z_m = 3649982.0;

  EsrExternalEmitterInput emitter;
  emitter.emitter_id = 2001U;
  emitter.emitter_name = "ten-kilometer-emitter";
  emitter.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  emitter.kinematics.position_ecef_m.x_m = -2289512.0;
  emitter.kinematics.position_ecef_m.y_m = 4919946.0;
  emitter.kinematics.position_ecef_m.z_m = 3645982.0;
  emitter.carrier_hz = 8.0e9;
  emitter.bandwidth_hz = 2.0e6;
  emitter.tx_power_w = 5.0e7;
  emitter.pulse_width_s = 1.0e-6;
  emitter.pri_s = 1.0e-4;

  EsrEnvironmentInput environment;
  environment.propagation_profile = EsrPropagationEnvironmentProfile::kTypical;
  environment.clutter_density = EsrClutterDensityLevel::kMedium;
  environment.spectrum_occupancy_ratio = 0.1f;

  EsrCycleInput input;
  EsrCoordinateStatus coordinate_status = EsrCoordinateStatus::kOk;
  ASSERT_TRUE(EsrCycleInputAdapter::Build(platform, {emitter}, 1.0f, environment, &input,
                                          &coordinate_status));
  input.cycle_index = 1U;

  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));
    EsrTraceSessionOptions options;
    options.replay_writer = replay_writer;
    EsrTraceSession session(config, options);
    const EsrCycleResult result = session.StepWithResult(input);
    ASSERT_EQ(result.status, EsrCycleExecutionStatus::kCompleted);
    ASSERT_EQ(result.output_frame.observation_output.observations.size(), 1U);
    replay_writer->Flush();
  }

  const EsrReplaySessionResult replay_result = ReplayEsrTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_EQ(replay_result.playback.applied_input_count, 1U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 1U);
  EXPECT_FALSE(replay_result.playback.divergence_found);
}

TEST(EsrReplaySessionTest, ReplayEsrTraceRejectsWrongModule) {
  const std::string trace_dir = MakeTempTracePath("oneq-esr-replay-wrong-module");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "esr-replay-wrong-module-test";
  manifest.module = "wrong_module";
  manifest.scenario_id = "unit-test";

  {
    oneq::replay::ReplayTraceWriter writer(trace_dir, manifest, true);
    writer.Flush();
  }

  const EsrReplaySessionResult replay_result = ReplayEsrTrace(trace_dir);
  EXPECT_FALSE(replay_result.ok);
}

TEST(EsrReplaySessionTest, ReplayEsrTraceContinuesAfterFailureMarker) {
  const std::string trace_dir = MakeTempTracePath("oneq-esr-replay-failure");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "esr-replay-failure-test";
  manifest.module = "electronic_surveillance_radar";
  manifest.scenario_id = "unit-test";

  {
    std::shared_ptr<oneq::replay::ReplayTraceWriter> replay_writer(
        new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));

    config::EsrSessionConfig config;
    config.hardware.beam_az_width_deg = 120.0f;
    config.hardware.beam_el_width_deg = 40.0f;

    EsrTraceSessionOptions options;
    options.replay_writer = replay_writer;
    options.trace_config_on_construct = true;
    EsrTraceSession session(config, options);

    EsrCycleInput input;
    input.cycle_index = 1U;
    input.dt_sec = 1.0f;
    session.StepWithResult(input);

    oneq::replay::ReplayTraceFailure failure;
    failure.error_code = "ESR_SIM_ASSERT";
    failure.message = "synthetic replay failure marker";
    failure.has_cycle_index = true;
    failure.cycle_index = 1U;
    const std::string failure_bytes = EncodeEsrFailureMarker(failure);
    replay_writer->WriteFailureMarker(failure, failure_bytes);
    input.cycle_index = 2U;
    session.StepWithResult(input);
    replay_writer->Flush();
  }

  const EsrReplaySessionResult replay_result = ReplayEsrTrace(trace_dir);
  EXPECT_TRUE(replay_result.ok) << replay_result.first_error;
  EXPECT_TRUE(replay_result.report.has_failure_marker);
  EXPECT_TRUE(replay_result.reached_failure_marker);
  EXPECT_EQ(replay_result.playback.failure_marker_count, 1U);
  EXPECT_EQ(replay_result.playback.applied_input_count, 2U);
  EXPECT_EQ(replay_result.playback.compared_output_count, 2U);
  EXPECT_EQ(replay_result.failure_marker_data.error_code, "ESR_SIM_ASSERT");
  EXPECT_EQ(replay_result.failure_marker_data.message, "synthetic replay failure marker");
  EXPECT_TRUE(replay_result.failure_marker_data.has_cycle_index);
}

TEST(EsrReplaySessionTest, ReplayEsrTraceRejectsTrailingCycleInput) {
  const std::string trace_dir = MakeTempTracePath("oneq-esr-replay-trailing-input");

  oneq::replay::ReplayTraceManifest manifest;
  manifest.trace_id = "esr-replay-trailing-input-test";
  manifest.module = "electronic_surveillance_radar";
  manifest.scenario_id = "unit-test";

  oneq::replay::ReplayTraceWriter writer(trace_dir, manifest, true);

  oneq::replay::ReplayTraceEvent config_event;
  config_event.module = "electronic_surveillance_radar";
  config_event.event_type = "session_config";
  config_event.payload_type = "EsrSessionConfig";
  config_event.payload_encoding = "flatbuffers";
  config_event.payload_bytes = EncodeEsrSessionConfig(config::EsrSessionConfig());
  writer.WriteEvent(config_event);

  EsrCycleInput input;
  input.cycle_index = 3U;
  input.dt_sec = 1.0f;
  oneq::replay::ReplayTraceEvent input_event;
  input_event.module = "electronic_surveillance_radar";
  input_event.event_type = "cycle_input";
  input_event.payload_type = "EsrCycleInput";
  input_event.payload_encoding = "flatbuffers";
  input_event.payload_bytes = EncodeEsrCycleInput(input);
  input_event.has_cycle_index = true;
  input_event.cycle_index = input.cycle_index;
  writer.WriteEvent(input_event);
  writer.Flush();

  const EsrReplaySessionResult replay_result = ReplayEsrTrace(trace_dir);
  EXPECT_FALSE(replay_result.ok);
  EXPECT_NE(replay_result.first_error.find("pending cycle_input"), std::string::npos);
}

}  // namespace tests
}  // namespace session
}  // namespace electronic_surveillance_radar
