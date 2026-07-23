#include <gtest/gtest.h>

#include <memory>

#include "1q/electromagnetics/RfScene.h"
#include "1q/replay/ReplayTrace.h"
#include "1q/electronic_surveillance_radar/session/EsrReplaySession.h"
#include "1q/electronic_surveillance_radar/session/EsrTraceSession.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

TEST(EsrReplaySessionTest, ReplaysDirectRfV2Input) {
  const std::string trace_dir = "/tmp/1q-esr-rf-v2-replay";
  oneq::replay::ReplayTraceManifest manifest;
  manifest.module = "electronic_surveillance_radar";
  std::shared_ptr<oneq::replay::ReplayTraceWriter> writer(
      new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));

  config::EsrSessionConfig config;
  config.policy.detection.minimum_snr_db = -100.0f;
  EsrTraceSessionOptions options;
  options.replay_writer = writer;
  EsrTraceSession session(config, options);
  EsrCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 10.0;
  input.dt_sec = 1.0f;
  input.platform_entity_id = 1U;
  input.has_platform_ecef_kinematics = true;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.interference.world_cycle_index = 1U;
  input.interference.window_start_time_s = 10.0;
  input.interference.window_duration_s = 1.0;
  ASSERT_EQ(session.StepWithResult(input).status, EsrCycleExecutionStatus::kCompleted);
  writer->Flush();

  const EsrReplaySessionResult replay = ReplayEsrTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
}

}  // namespace
}  // namespace session
}  // namespace electronic_surveillance_radar
