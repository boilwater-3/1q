// Copyright 2026. All Rights Reserved.
//
// @file ar_public_api_convenience_test.cpp
// @brief 验证 AR 单周期用户门面的可用性、失败原子性与 RF 交换合同。

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "1q/airborne_radar/config/ArRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/ArSessionConfigBuilder.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/coordinate/position_transform.h"
#include "1q/electronic_countermeasure/EcmTypes.h"

namespace airborne_radar {
namespace tests {
namespace {

config::ArSessionConfig MakeSessionConfig() {
  return config::ArSessionConfigBuilder()
      .Detection()
      .WithDetectionIntentProfile(
          config::profiles::DetectionIntentProfile::kDetectionPriority)
      .End()
      .Tracking()
      .WithTrackingPolicyProfile(
          config::profiles::TrackingPolicyProfile::kFastAssociation)
      .End()
      .Lifecycle()
      .WithLifecyclePolicyProfile(
          config::profiles::LifecyclePolicyProfile::kFastConfirm)
      .End()
      .Build();
}

session::ArCycleInput MakeInput(std::uint32_t cycle_index = 1U,
                                double cycle_start_time_s = 0.0) {
  session::ArCycleInput input;
  input.cycle_index = cycle_index;
  input.cycle_start_time_s = cycle_start_time_s;
  input.dt_sec = 0.5;
  input.platform.platform_entity_id = 42U;

  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 31.0;
  platform_lla.longitude_deg = 121.0;
  platform_lla.altitude_m = 1000.0;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(
      platform_lla, &input.platform.platform_position_ecef_m));

  session::ArTargetInput target;
  target.target_id = 7U;
  target.target_name = "contract-target";
  target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  target.kinematics.position_ecef_m = input.platform.platform_position_ecef_m;
  target.kinematics.position_ecef_m.x_m += 5000.0;
  target.rcs = 2.0f;
  input.targets.push_back(target);
  return input;
}

oneq::electromagnetics::RfSceneEmission MakeNoiseEmission(
    const session::ArCycleInput& input, std::uint64_t emission_id, double frequency_hz,
    double bandwidth_hz, double transmit_power_w) {
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.platform_id = 99U;
  emission.identity.equipment_id = 5U;
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m = input.platform.platform_position_ecef_m;
  emission.position_ecef_m.x_m += 10000.0;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(
      input.cycle_start_time_s, input.dt_sec, frequency_hz, bandwidth_hz,
      transmit_power_w, &emission.waveform));
  return emission;
}

void SetInterferenceFrame(
    const session::ArCycleInput& input,
    const std::vector<oneq::electromagnetics::RfSceneEmission>& emissions,
    oneq::electromagnetics::RfEmissionFrame* frame) {
  ASSERT_NE(frame, nullptr);
  frame->world_cycle_index = input.cycle_index;
  frame->window_start_time_s = input.cycle_start_time_s;
  frame->window_duration_s = input.dt_sec;
  frame->emissions = emissions;
}

}  // namespace

TEST(ArPublicApiContractTest, UserInputContainsWorldFactsAndIndependentInterference) {
  const session::ArCycleInput input = MakeInput();
  EXPECT_EQ(input.platform.platform_entity_id, 42U);
  ASSERT_EQ(input.targets.size(), 1U);
  EXPECT_EQ(input.targets.front().target_id, 7U);
  EXPECT_TRUE(input.interference.emissions.empty());
}

TEST(ArPublicApiContractTest, StepWithResultCompletesAndPublishesActualEmission) {
  session::ArSession radar = session::ArSession::Create(MakeSessionConfig());
  const session::ArCycleResult result = radar.StepWithResult(MakeInput());

  ASSERT_EQ(result.status, session::ArCycleStatus::kCompleted);
  EXPECT_FALSE(result.has_validation_error);
  EXPECT_EQ(result.input_cycle_index, 1U);
  EXPECT_EQ(result.emission_frame.world_cycle_index, 1U);
  ASSERT_EQ(result.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(result.emission_frame.emissions.front().identity.platform_id, 42U);
  EXPECT_EQ(result.emission_frame.emissions.front().identity.emission_id, 1U);
}

TEST(ArPublicApiContractTest, StepReturnsSameTracksAsEquivalentStepWithResult) {
  session::ArSession step_radar = session::ArSession::Create(MakeSessionConfig());
  session::ArSession result_radar = session::ArSession::Create(MakeSessionConfig());

  const session::TrackOutputFrame step_frame = step_radar.Step(MakeInput());
  const session::ArCycleResult result = result_radar.StepWithResult(MakeInput());

  ASSERT_EQ(result.status, session::ArCycleStatus::kCompleted);
  EXPECT_EQ(step_frame.cycle_index, result.track_output_frame.cycle_index);
  EXPECT_EQ(step_frame.batch_id, result.track_output_frame.batch_id);
  EXPECT_EQ(step_frame.tracks.size(), result.track_output_frame.tracks.size());
}

TEST(ArPublicApiContractTest, RejectedCycleDoesNotReusePreviousOutput) {
  session::ArSession radar = session::ArSession::Create(MakeSessionConfig());
  ASSERT_EQ(radar.StepWithResult(MakeInput()).status,
            session::ArCycleStatus::kCompleted);

  session::ArCycleInput invalid = MakeInput(2U, 0.5);
  invalid.dt_sec = 0.0;
  const session::ArCycleResult rejected = radar.StepWithResult(invalid);

  EXPECT_EQ(rejected.status, session::ArCycleStatus::kRejectedInvalidInput);
  EXPECT_TRUE(rejected.has_validation_error);
  EXPECT_TRUE(rejected.track_output_frame.tracks.empty());
  EXPECT_TRUE(rejected.emission_frame.emissions.empty());
}

TEST(ArPublicApiContractTest, StepReturnsEmptyCurrentFrameOnRejectedInput) {
  session::ArSession radar = session::ArSession::Create(MakeSessionConfig());
  ASSERT_FALSE(radar.Step(MakeInput()).tracks.empty());

  session::ArCycleInput invalid = MakeInput(2U, 0.5);
  invalid.cycle_index = 0U;
  const session::TrackOutputFrame rejected = radar.Step(invalid);

  EXPECT_TRUE(rejected.tracks.empty());
  EXPECT_EQ(rejected.cycle_index, 0U);
}

TEST(ArPublicApiContractTest, RejectedInputDoesNotConsumeEmissionIdentity) {
  session::ArSession radar = session::ArSession::Create(MakeSessionConfig());
  session::ArCycleInput invalid = MakeInput();
  invalid.platform.platform_entity_id = 0U;
  ASSERT_EQ(radar.StepWithResult(invalid).status,
            session::ArCycleStatus::kRejectedInvalidInput);

  const session::ArCycleResult accepted = radar.StepWithResult(MakeInput());
  ASSERT_EQ(accepted.status, session::ArCycleStatus::kCompleted);
  ASSERT_EQ(accepted.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(accepted.emission_frame.emissions.front().identity.emission_id, 1U);
}

TEST(ArPublicApiContractTest, MismatchedInterferenceFrameRejectsAndCanRetrySameCycle) {
  session::ArSession radar = session::ArSession::Create(MakeSessionConfig());
  session::ArCycleInput mismatch = MakeInput();
  SetInterferenceFrame(
      mismatch,
      {MakeNoiseEmission(mismatch, 10U, MakeSessionConfig().hardware.transmitter.frequency_hz,
                         5.0e6, 100.0)},
      &mismatch.interference);
  mismatch.interference.world_cycle_index = 2U;
  ASSERT_EQ(radar.StepWithResult(mismatch).status,
            session::ArCycleStatus::kRejectedInvalidInput);

  const session::ArCycleResult retried = radar.StepWithResult(MakeInput());
  ASSERT_EQ(retried.status, session::ArCycleStatus::kCompleted);
  ASSERT_EQ(retried.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(retried.emission_frame.emissions.front().identity.emission_id, 1U);
}

TEST(ArPublicApiContractTest, EcmEmissionFrameAssignsDirectlyToArInterference) {
  electronic_countermeasure::session::EcmCycleResult ecm_result;
  session::ArCycleInput ar_input = MakeInput();
  ar_input.interference = ecm_result.emission_frame;
  EXPECT_TRUE(ar_input.interference.emissions.empty());
}

TEST(ArPublicApiContractTest, MultipleEngineeringSourcesUseOnePublicFrame) {
  session::ArCycleInput input = MakeInput();
  const double radar_frequency_hz =
      static_cast<double>(MakeSessionConfig().hardware.transmitter.frequency_hz);
  SetInterferenceFrame(
      input,
      {MakeNoiseEmission(input, 10U, radar_frequency_hz, 5.0e6, 100.0),
       MakeNoiseEmission(input, 11U, radar_frequency_hz, 10.0e6, 200.0)},
      &input.interference);
  input.interference.emissions.back().identity.platform_id = 100U;

  session::ArSession radar = session::ArSession::Create(MakeSessionConfig());
  const session::ArCycleResult result = radar.StepWithResult(input);
  EXPECT_EQ(result.status, session::ArCycleStatus::kCompleted);
}

TEST(ArPublicApiContractTest, ReceiverSaturationIsACompletedStructuredImpairment) {
  session::ArCycleInput input = MakeInput();
  const double radar_frequency_hz =
      static_cast<double>(MakeSessionConfig().hardware.transmitter.frequency_hz);
  SetInterferenceFrame(
      input,
      {MakeNoiseEmission(input, 10U, radar_frequency_hz, 20.0e6, 1.0e18)},
      &input.interference);

  session::ArSession radar = session::ArSession::Create(MakeSessionConfig());
  const session::ArCycleResult result = radar.StepWithResult(input);

  EXPECT_EQ(result.status, session::ArCycleStatus::kCompleted);
  EXPECT_EQ(result.receiver_impairment,
            session::ArReceiverImpairment::kSaturated);
  EXPECT_TRUE(result.interference_observations.empty());
}

TEST(ArPublicApiContractTest, InvalidNaturalEnvironmentRejectsBeforeRfStateChanges) {
  session::ArSession radar = session::ArSession::Create(MakeSessionConfig());
  session::ArCycleInput input = MakeInput();
  input.environment.atmospheric_observation.pressure_hpa =
      std::numeric_limits<float>::quiet_NaN();
  const session::ArCycleResult rejected = radar.StepWithResult(input);
  EXPECT_EQ(rejected.status, session::ArCycleStatus::kRejectedInvalidInput);

  const session::ArCycleResult retried = radar.StepWithResult(MakeInput());
  ASSERT_EQ(retried.status, session::ArCycleStatus::kCompleted);
  ASSERT_EQ(retried.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(retried.emission_frame.emissions.front().identity.emission_id, 1U);
}

TEST(ArPublicApiContractTest, PoweredOffCycleAdvancesTimeWithoutEmission) {
  session::ArSession radar = session::ArSession::Create(MakeSessionConfig());
  const config::ArRuntimeConfigPatch power_off =
      config::ArRuntimeConfigBuilder().WithSensorEnabled(false).Build();
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(power_off));

  const session::ArCycleResult off = radar.StepWithResult(MakeInput());
  EXPECT_EQ(off.status, session::ArCycleStatus::kPoweredOff);
  EXPECT_TRUE(off.emission_frame.emissions.empty());

  const config::ArRuntimeConfigPatch power_on =
      config::ArRuntimeConfigBuilder().WithSensorEnabled(true).Build();
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(power_on));
  const session::ArCycleResult on = radar.StepWithResult(MakeInput(2U, 0.5));
  ASSERT_EQ(on.status, session::ArCycleStatus::kCompleted);
  ASSERT_EQ(on.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(on.emission_frame.emissions.front().identity.emission_id, 1U);
}

TEST(ArPublicApiContractTest, WorldTimeRegressionRejectsWithoutHistoricalOutput) {
  session::ArSession radar = session::ArSession::Create(MakeSessionConfig());
  ASSERT_EQ(radar.StepWithResult(MakeInput(1U, 1.0)).status,
            session::ArCycleStatus::kCompleted);

  const session::ArCycleResult rejected =
      radar.StepWithResult(MakeInput(2U, 1.25));
  EXPECT_EQ(rejected.status, session::ArCycleStatus::kRejectedInvalidConfig);
  EXPECT_TRUE(rejected.track_output_frame.tracks.empty());
  EXPECT_TRUE(rejected.emission_frame.emissions.empty());
}

}  // namespace tests
}  // namespace airborne_radar
