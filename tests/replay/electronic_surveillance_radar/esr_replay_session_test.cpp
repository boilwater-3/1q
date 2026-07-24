#include <gtest/gtest.h>

#include <memory>

#include "1q/electromagnetics/RfScene.h"
#include "1q/replay/ReplayTrace.h"
#include "1q/electronic_surveillance_radar/session/EsrReplaySession.h"
#include "1q/electronic_surveillance_radar/session/EsrTraceSession.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

EsrCycleInput MakeWaveformClassInput(
    std::uint32_t cycle_index,
    oneq::electromagnetics::RfSceneWaveformKind waveform_kind) {
  EsrCycleInput input;
  input.cycle_index = cycle_index;
  input.cycle_start_time_s = static_cast<double>(cycle_index - 1U);
  input.dt_sec = 1.0f;
  input.platform_entity_id = 1U;
  input.has_platform_ecef_kinematics = true;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.platform_id = 2U;
  emission.identity.equipment_id = 3U;
  emission.identity.emission_id = cycle_index;
  emission.position_ecef_m.x_m = 6378137.0;
  emission.position_ecef_m.y_m = 1000.0;
  emission.antenna.boresight_ecef.y = -1.0;
  if (waveform_kind ==
      oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain) {
    EXPECT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
        input.cycle_start_time_s, 10.0e9, 1.0e6, 1.0e6, 1.0e-6, 1.0e-3,
        1000U, 0.0, 1U, cycle_index, &emission.waveform));
  } else {
    EXPECT_TRUE(oneq::electromagnetics::TryCreateRfContinuousWaveform(
        input.cycle_start_time_s, input.dt_sec, 10.0e9, 1.0e6, 1.0e6,
        &emission.waveform));
  }
  input.rf_emissions.emissions.push_back(emission);
  return input;
}

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
  input.rf_emissions.world_cycle_index = 1U;
  input.rf_emissions.window_start_time_s = 10.0;
  input.rf_emissions.window_duration_s = 1.0;
  ASSERT_EQ(session.StepWithResult(input).status, EsrCycleExecutionStatus::kCompleted);
  writer->Flush();

  const EsrReplaySessionResult replay = ReplayEsrTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
}

TEST(EsrReplaySessionTest, WaveformClassGateContinuesDeterministicallyInReplay) {
  const std::string trace_dir = "/tmp/1q-esr-waveform-class-replay";
  oneq::replay::ReplayTraceManifest manifest;
  manifest.module = "electronic_surveillance_radar";
  std::shared_ptr<oneq::replay::ReplayTraceWriter> writer(
      new oneq::replay::ReplayTraceWriter(trace_dir, manifest, true));

  config::EsrSessionConfig config;
  config.hardware.receiver_band_lower_hz = 9.99e9;
  config.hardware.receiver_band_upper_hz = 10.01e9;
  config.hardware.beam_az_width_deg = 120.0f;
  config.hardware.beam_el_width_deg = 120.0f;
  config.hardware.maximum_linear_input_power_w = 10.0f;
  config.policy.detection.minimum_snr_db = -100.0f;
  config.policy.detection.enable_statistical_detection = false;
  EsrTraceSessionOptions options;
  options.replay_writer = writer;
  EsrTraceSession session(config, options);

  const EsrCycleResult pulse_result = session.StepWithResult(MakeWaveformClassInput(
      1U, oneq::electromagnetics::RfSceneWaveformKind::kPulseTrain));
  ASSERT_EQ(pulse_result.status, EsrCycleExecutionStatus::kCompleted);
  ASSERT_EQ(pulse_result.output_frame.emitter_output.hypotheses.size(), 1U);
  const EsrCycleResult continuous_result =
      session.StepWithResult(MakeWaveformClassInput(
          2U, oneq::electromagnetics::RfSceneWaveformKind::kContinuous));
  ASSERT_EQ(continuous_result.status, EsrCycleExecutionStatus::kCompleted);
  ASSERT_EQ(continuous_result.output_frame.emitter_output.hypotheses.size(), 2U);
  EXPECT_NE(continuous_result.output_frame.emitter_output.hypotheses[0].waveform_class,
            continuous_result.output_frame.emitter_output.hypotheses[1].waveform_class);
  writer->Flush();

  const EsrReplaySessionResult replay = ReplayEsrTrace(trace_dir);
  EXPECT_TRUE(replay.ok) << replay.first_error;
  EXPECT_EQ(replay.playback.compared_output_count, 2U);
}

}  // namespace
}  // namespace session
}  // namespace electronic_surveillance_radar
