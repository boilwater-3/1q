#include <gtest/gtest.h>

#include <cmath>

#include "1q/electromagnetics/RfScene.h"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

config::EsrSessionConfig MakeConfig() {
  config::EsrSessionConfig config;
  config.policy.detection.minimum_snr_db = -100.0f;
  config.policy.detection.enable_statistical_detection = false;
  config.hardware.receiver_band_lower_hz = 9.0e9;
  config.hardware.receiver_band_upper_hz = 11.0e9;
  config.hardware.beam_az_width_deg = 180.0f;
  config.hardware.beam_el_width_deg = 180.0f;
  return config;
}

EsrCycleInput MakeInput() {
  EsrCycleInput input;
  input.cycle_index = 1U;
  input.cycle_start_time_s = 10.0;
  input.dt_sec = 1.0f;
  input.platform_entity_id = 1U;
  input.has_platform_ecef_kinematics = true;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;
  return input;
}

oneq::electromagnetics::RfSceneEmission MakeEmission(std::uint64_t emission_id, double power_w) {
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.platform_id = 100U + emission_id;
  emission.identity.equipment_id = 200U + emission_id;
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m.x_m = 6378137.0;
  emission.position_ecef_m.y_m = 1000.0;
  emission.antenna.boresight_ecef.y = -1.0;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(
      10.0, 1.0, 10.0e9, 1.0e6, power_w, &emission.waveform));
  return emission;
}

TEST(EsrSessionIntegrationTest, StepWithResultConsumesDirectRfV2Frame) {
  EsrCycleInput input = MakeInput();
  input.rf_emissions.emissions.push_back(MakeEmission(1U, 1.0e6));
  const EsrCycleResult result = EsrSession::Create(MakeConfig()).StepWithResult(input);
  EXPECT_EQ(result.status, EsrCycleExecutionStatus::kCompleted);
  EXPECT_FALSE(result.has_validation_error);
  EXPECT_EQ(result.output_frame.cycle_index, input.cycle_index);
  EXPECT_NE(result.abort_reason, EsrPipelineAbortReason::kRfReceiverRejected);
}

TEST(EsrSessionIntegrationTest, InvalidCoSitePathRejectsWithoutOutput) {
  EsrCycleInput input = MakeInput();
  oneq::electromagnetics::RfSceneEmission emission = MakeEmission(1U, 1.0e6);
  emission.identity.platform_id = input.platform_entity_id;
  input.rf_emissions.emissions.push_back(emission);
  const EsrCycleResult result = EsrSession::Create(MakeConfig()).StepWithResult(input);
  EXPECT_EQ(result.status, EsrCycleExecutionStatus::kRejected);
  EXPECT_EQ(result.abort_reason, EsrPipelineAbortReason::kRfReceiverRejected);
  EXPECT_EQ(result.output_frame.cycle_index, 0U);
}

TEST(EsrSessionIntegrationTest,
     PolarBearingSingularityDoesNotRejectObservablePeer) {
  EsrCycleInput input = MakeInput();
  oneq::electromagnetics::RfSceneEmission polar = MakeEmission(1U, 1.0e6);
  polar.position_ecef_m.x_m += 1000.0;
  polar.position_ecef_m.y_m = 0.0;
  polar.antenna.boresight_ecef.x = -1.0;
  polar.antenna.boresight_ecef.y = 0.0;
  input.rf_emissions.emissions.push_back(polar);
  input.rf_emissions.emissions.push_back(MakeEmission(2U, 1.0e6));

  const EsrCycleResult result =
      EsrSession::Create(MakeConfig()).StepWithResult(input);
  EXPECT_EQ(result.status, EsrCycleExecutionStatus::kCompleted);
  EXPECT_EQ(result.abort_reason, EsrPipelineAbortReason::kNone);
  ASSERT_EQ(result.output_frame.observation_output.observations.size(), 1U);
  EXPECT_LT(
      std::abs(result.output_frame.observation_output.observations.front().aoa_el_deg),
      30.0);
}

TEST(EsrSessionIntegrationTest, PowerOffProducesNoHistoricalOutput) {
  EsrSession session = EsrSession::Create(MakeConfig());
  EsrCycleInput input = MakeInput();
  input.rf_emissions.emissions.push_back(MakeEmission(1U, 1.0e6));
  ASSERT_EQ(session.StepWithResult(input).status, EsrCycleExecutionStatus::kCompleted);
  (void)session.TryApplyRuntimeConfig(config::EsrRuntimeConfigBuilder().WithSensorEnabled(false).Build());
  const EsrCycleResult powered_off = session.StepWithResult(input);
  EXPECT_EQ(powered_off.status, EsrCycleExecutionStatus::kPoweredOff);
  EXPECT_EQ(powered_off.output_frame.cycle_index, 0U);
}

}  // namespace
}  // namespace session
}  // namespace electronic_surveillance_radar
