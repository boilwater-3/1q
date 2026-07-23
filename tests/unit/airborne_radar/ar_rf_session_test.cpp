#include <gtest/gtest.h>

#include "1q/airborne_radar/config/ArRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/ArSessionConfigBuilder.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/session/ControlDirective.h"
#include "1q/coordinate/position_transform.h"

namespace airborne_radar {
namespace session {
namespace {

config::ArSessionConfig MakeRfConfig() {
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

ArCycleInput MakeInput(std::uint32_t cycle, double start_time_s) {
  ArCycleInput input;
  input.cycle_index = cycle;
  input.cycle_start_time_s = start_time_s;
  input.dt_sec = 0.5;
  input.platform.platform_entity_id = 10U;
  oneq::coordinate::LlaPositionDegM platform_lla;
  platform_lla.latitude_deg = 31.0;
  platform_lla.longitude_deg = 121.0;
  platform_lla.altitude_m = 1000.0;
  EXPECT_TRUE(oneq::coordinate::TryLlaToEcef(
      platform_lla, &input.platform.platform_position_ecef_m));
  ArTargetInput target;
  target.target_id = 77U;
  target.kinematics.position_frame = oneq::coordinate::PositionFrame::kEcef;
  target.kinematics.position_ecef_m =
      input.platform.platform_position_ecef_m;
  target.kinematics.position_ecef_m.x_m += 5000.0;
  target.rcs = 5.0f;
  input.targets.push_back(target);
  return input;
}

void AddNoiseInterference(double transmit_power_w, ArCycleInput* input) {
  ASSERT_NE(input, nullptr);
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity = {99U, 3U, 1U};
  emission.position_ecef_m = input->platform.platform_position_ecef_m;
  emission.position_ecef_m.x_m += 10000.0;
  const double radar_frequency_hz =
      static_cast<double>(MakeRfConfig().hardware.transmitter.frequency_hz);
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(
      input->cycle_start_time_s, input->dt_sec, radar_frequency_hz, 20.0e6,
      transmit_power_w, &emission.waveform));
  input->interference.world_cycle_index = input->cycle_index;
  input->interference.window_start_time_s = input->cycle_start_time_s;
  input->interference.window_duration_s = input->dt_sec;
  input->interference.emissions.push_back(emission);
}

TEST(ArRfSessionTest, SaturationCompletesWithoutFalseRfObservation) {
  ArCycleInput input = MakeInput(1U, 0.0);
  AddNoiseInterference(1.0e18, &input);
  ArSession radar = ArSession::Create(MakeRfConfig());

  const ArCycleResult result = radar.StepWithResult(input);

  EXPECT_EQ(result.status, ArCycleStatus::kCompleted);
  EXPECT_EQ(result.receiver_impairment, ArReceiverImpairment::kSaturated);
  EXPECT_TRUE(result.interference_observations.empty());
  EXPECT_TRUE(result.track_output_frame.tracks.empty());
}

TEST(ArRfSessionTest, ReceiverObservationContainsNoTruthIdentity) {
  ArCycleInput input = MakeInput(1U, 0.0);
  AddNoiseInterference(1.0e8, &input);
  ArSession radar = ArSession::Create(MakeRfConfig());

  const ArCycleResult result = radar.StepWithResult(input);

  ASSERT_EQ(result.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(result.receiver_impairment, ArReceiverImpairment::kNone);
  ASSERT_FALSE(result.interference_observations.empty());
  EXPECT_GT(result.interference_observations.front().jammer_to_noise_db, 0.0);
  EXPECT_NE(result.interference_observations.front().observation_id, 0U);
}

TEST(ArRfSessionTest, ExternalAgilityDecisionChangesNextActualCarrier) {
  config::ArSessionConfig config;
  config.hardware.transmitter.frequency_plan_hz = {3.0e9, 3.1e9};
  ArSession radar = ArSession::Create(config);
  const ArCycleResult first = radar.StepWithResult(MakeInput(1U, 0.0));
  ASSERT_EQ(first.status, ArCycleStatus::kCompleted);
  ASSERT_TRUE(first.has_decision_observation);

  ExternalDecisionResponse response;
  response.source_cycle_index =
      first.decision_observation.input_frame.cycle_index;
  response.source_batch_id = first.decision_observation.input_frame.batch_id;
  response.proposals.push_back(TacticalProposal{
      ControlDirective(ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                       ControlDirectiveSource::SURVIVABILITY),
      90, "agility"});
  ASSERT_EQ(radar.SubmitExternalDecision(response),
            ExternalDecisionSubmitStatus::kAccepted);

  const ArCycleResult second = radar.StepWithResult(MakeInput(2U, 0.5));
  ASSERT_EQ(second.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(second.emission_frame.emissions.size(), 1U);
  EXPECT_DOUBLE_EQ(
      second.emission_frame.emissions.front().waveform.center_frequency_hz,
      3.1e9);
}

TEST(ArRfSessionTest, RuntimePointingPatchChangesNextActualBoresight) {
  ArSession radar = ArSession::Create();
  const ArCycleResult first = radar.StepWithResult(MakeInput(1U, 0.0));
  ASSERT_EQ(first.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(first.emission_frame.emissions.size(), 1U);

  config::AzimuthElevationDeg scan_center;
  scan_center.az_deg = 20.0f;
  const config::ArRuntimeConfigPatch patch =
      config::ArRuntimeConfigBuilder()
          .WithScanCenterDeg(scan_center)
          .Build();
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(patch));

  const ArCycleResult second = radar.StepWithResult(MakeInput(2U, 0.5));
  ASSERT_EQ(second.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(second.emission_frame.emissions.size(), 1U);
  const oneq::electromagnetics::RfSceneDirection& before =
      first.emission_frame.emissions.front().antenna.boresight_ecef;
  const oneq::electromagnetics::RfSceneDirection& after =
      second.emission_frame.emissions.front().antenna.boresight_ecef;
  EXPECT_TRUE(before.x != after.x || before.y != after.y ||
              before.z != after.z);
}

TEST(ArRfSessionTest, PoweredOffCycleDoesNotConsumeEmissionIdentity) {
  ArSession radar = ArSession::Create();
  ASSERT_TRUE(radar.TryApplyRuntimeConfig(
      config::ArRuntimeConfigBuilder().WithSensorEnabled(false).Build()));
  EXPECT_EQ(radar.StepWithResult(MakeInput(1U, 0.0)).status,
            ArCycleStatus::kPoweredOff);

  ASSERT_TRUE(radar.TryApplyRuntimeConfig(
      config::ArRuntimeConfigBuilder().WithSensorEnabled(true).Build()));
  const ArCycleResult on = radar.StepWithResult(MakeInput(2U, 0.5));
  ASSERT_EQ(on.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(on.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(on.emission_frame.emissions.front().identity.emission_id, 1U);
}

}  // namespace
}  // namespace session
}  // namespace airborne_radar
