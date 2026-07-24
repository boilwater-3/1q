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

void AddUnconfiguredCoSiteInterference(ArCycleInput* input) {
  ASSERT_NE(input, nullptr);
  AddNoiseInterference(1.0, input);
  oneq::electromagnetics::RfSceneEmission& emission = input->interference.emissions.back();
  emission.identity.platform_id = input->platform.platform_entity_id;
  emission.position_ecef_m = input->platform.platform_position_ecef_m;
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

TEST(ArRfSessionTest, ReceiveRejectionCommitsEmissionIdentityChronologyAndAppliedAgility) {
  config::ArSessionConfig config;
  config.hardware.transmitter.frequency_plan_hz = {3.0e9, 3.1e9};
  ArSession radar = ArSession::Create(config);
  const ArCycleResult first = radar.StepWithResult(MakeInput(1U, 0.0));
  ASSERT_EQ(first.status, ArCycleStatus::kCompleted);
  ASSERT_TRUE(first.has_decision_observation);

  ExternalDecisionResponse response;
  response.source_cycle_index = first.decision_observation.input_frame.cycle_index;
  response.source_batch_id = first.decision_observation.input_frame.batch_id;
  response.proposals.push_back(
      TacticalProposal{ControlDirective(ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
                                        ControlDirectiveSource::SURVIVABILITY),
                       90, "agility"});
  ASSERT_EQ(radar.SubmitExternalDecision(response), ExternalDecisionSubmitStatus::kAccepted);

  ArCycleInput rejected_input = MakeInput(2U, 0.5);
  AddUnconfiguredCoSiteInterference(&rejected_input);
  const ArCycleResult rejected = radar.StepWithResult(rejected_input);
  ASSERT_EQ(rejected.status, ArCycleStatus::kRejectedExecution);
  ASSERT_EQ(rejected.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(rejected.emission_frame.emissions.front().identity.emission_id, 2U);
  EXPECT_DOUBLE_EQ(rejected.emission_frame.emissions.front().waveform.center_frequency_hz, 3.1e9);
  EXPECT_EQ(rejected.applied_decision_source, DecisionControlSource::kExternal);

  const ArCycleResult next = radar.StepWithResult(MakeInput(3U, 1.0));
  ASSERT_EQ(next.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(next.emission_frame.emissions.size(), 1U);
  EXPECT_EQ(next.emission_frame.emissions.front().identity.emission_id, 3U);
  EXPECT_DOUBLE_EQ(next.emission_frame.emissions.front().waveform.center_frequency_hz, 3.0e9);
}

TEST(ArRfSessionTest, EccmSidelobeControlsKeepNextReceiverPatternValid) {
  ArSession radar = ArSession::Create(MakeRfConfig());
  const ArCycleResult first = radar.StepWithResult(MakeInput(1U, 0.0));
  ASSERT_EQ(first.status, ArCycleStatus::kCompleted);
  ASSERT_TRUE(first.has_decision_observation);

  ExternalDecisionResponse response;
  response.source_cycle_index = first.decision_observation.input_frame.cycle_index;
  response.source_batch_id = first.decision_observation.input_frame.batch_id;
  response.proposals.push_back(
      TacticalProposal{ControlDirective(ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER,
                                        ControlDirectiveSource::SURVIVABILITY),
                       90, "sidelobe-canceller"});
  response.proposals.push_back(
      TacticalProposal{ControlDirective(ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
                                        ControlDirectiveSource::SURVIVABILITY),
                       80, "adaptive-beamforming"});
  ASSERT_EQ(radar.SubmitExternalDecision(response), ExternalDecisionSubmitStatus::kAccepted);

  const ArCycleResult second = radar.StepWithResult(MakeInput(2U, 0.5));
  EXPECT_EQ(second.status, ArCycleStatus::kCompleted);
}

TEST(ArRfSessionTest, InertialStabilizationKeepsActualEcefBoresightFixed) {
  config::ArSessionConfig body_config = MakeRfConfig();
  body_config.mission.orientation.work_mode = config::ArWorkMode::kStt;
  body_config.mission.orientation.stabilization_mode = config::StabilizationMode::kBodyStabilized;
  config::ArSessionConfig inertial_config = body_config;
  inertial_config.mission.orientation.stabilization_mode =
      config::StabilizationMode::kInertialStabilized;

  ArCycleInput level_input = MakeInput(1U, 0.0);
  ArCycleInput yawed_input = level_input;
  yawed_input.platform.platform_attitude_deg.yaw_deg = 30.0;

  ArSession body_level = ArSession::Create(body_config);
  ArSession body_yawed = ArSession::Create(body_config);
  ArSession inertial_level = ArSession::Create(inertial_config);
  ArSession inertial_yawed = ArSession::Create(inertial_config);
  const ArCycleResult body_level_result = body_level.StepWithResult(level_input);
  const ArCycleResult body_yawed_result = body_yawed.StepWithResult(yawed_input);
  const ArCycleResult inertial_level_result = inertial_level.StepWithResult(level_input);
  const ArCycleResult inertial_yawed_result = inertial_yawed.StepWithResult(yawed_input);

  ASSERT_EQ(body_level_result.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(body_yawed_result.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(inertial_level_result.status, ArCycleStatus::kCompleted);
  ASSERT_EQ(inertial_yawed_result.status, ArCycleStatus::kCompleted);
  const auto& body_level_boresight =
      body_level_result.emission_frame.emissions.front().antenna.boresight_ecef;
  const auto& body_yawed_boresight =
      body_yawed_result.emission_frame.emissions.front().antenna.boresight_ecef;
  const auto& inertial_level_boresight =
      inertial_level_result.emission_frame.emissions.front().antenna.boresight_ecef;
  const auto& inertial_yawed_boresight =
      inertial_yawed_result.emission_frame.emissions.front().antenna.boresight_ecef;
  const double body_delta_squared = (body_level_boresight.x - body_yawed_boresight.x) *
                                        (body_level_boresight.x - body_yawed_boresight.x) +
                                    (body_level_boresight.y - body_yawed_boresight.y) *
                                        (body_level_boresight.y - body_yawed_boresight.y) +
                                    (body_level_boresight.z - body_yawed_boresight.z) *
                                        (body_level_boresight.z - body_yawed_boresight.z);
  const double inertial_delta_squared =
      (inertial_level_boresight.x - inertial_yawed_boresight.x) *
          (inertial_level_boresight.x - inertial_yawed_boresight.x) +
      (inertial_level_boresight.y - inertial_yawed_boresight.y) *
          (inertial_level_boresight.y - inertial_yawed_boresight.y) +
      (inertial_level_boresight.z - inertial_yawed_boresight.z) *
          (inertial_level_boresight.z - inertial_yawed_boresight.z);
  EXPECT_GT(body_delta_squared, 0.1);
  EXPECT_LT(inertial_delta_squared, 1.0e-10);
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
