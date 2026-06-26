/**
 * @file public_headers_smoke_test.cpp
 * @brief 验证稳定公共头集合可被统一包含并完成最小用法编译。
 */

#include <gtest/gtest.h>

#include <memory>
#include <type_traits>
#include <utility>

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/airborne_radar/config/RadarEnvironmentConfig.h"
#include "1q/airborne_radar/config/RadarHardwareConfig.h"
#include "1q/airborne_radar/config/RadarMissionConfig.h"
#include "1q/airborne_radar/config/RadarPolicyConfig.h"
#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/RadarRuntimeConfigPatch.h"
#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/config/airborne_radar_config.hpp"
#include "1q/airborne_radar/environment/EnvironmentConfig.h"
#include "1q/airborne_radar/environment/EnvironmentRuntimeConfigPatch.h"
#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"
#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/environment/airborne_radar_environment.hpp"
#include "1q/airborne_radar/extension/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/extension/SignalPipelineResultTypes.h"
#include "1q/airborne_radar/extension/airborne_radar_extension.hpp"
#include "1q/airborne_radar/extension/control/ControlDirective.h"
#include "1q/airborne_radar/extension/control/RadarCommand.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/model/DecisionInputFrame.h"
#include "1q/airborne_radar/model/DecisionSourceInfo.h"
#include "1q/airborne_radar/model/JammingSemantics.h"
#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/airborne_radar/model/TargetCategory.h"
#include "1q/airborne_radar/model/TrackStateSnapshot.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarCycleInputBuilder.h"
#include "1q/airborne_radar/session/RadarCycleResult.h"
#include "1q/airborne_radar/session/RadarEnvironmentInput.h"
#include "1q/airborne_radar/session/RadarEnvironmentInputPatch.h"
#include "1q/airborne_radar/session/RadarEnvironmentInputState.h"
#include "1q/airborne_radar/session/RadarInputValidation.h"

#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"
#include "1q/airborne_radar/session/RadarTraceSession.h"
#include "1q/airborne_radar/session/RadarTrackLifecycleRecorder.h"
#include "1q/airborne_radar/session/RadarTrackOutputDebugView.h"
#include "1q/api.hpp"
#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/config/electro_optical_sensor_config.hpp"
#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"
#include "1q/electro_optical_sensor/environment/EosEnvironmentConfig.h"
#include "1q/electro_optical_sensor/environment/EosEnvironmentRuntimeConfigPatch.h"
#include "1q/electro_optical_sensor/environment/EosEnvironmentTypes.h"
#include "1q/electro_optical_sensor/environment/electro_optical_sensor_environment.hpp"
#include "1q/electro_optical_sensor/extension/EosPipelineTypes.h"
#include "1q/electro_optical_sensor/extension/electro_optical_sensor_extension.hpp"
#include "1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleInputBuilder.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosDetectionLifecycleRecorder.h"
#include "1q/electro_optical_sensor/session/EosEnvironmentInput.h"
#include "1q/electro_optical_sensor/session/EosEnvironmentInputPatch.h"
#include "1q/electro_optical_sensor/session/EosEnvironmentInputState.h"
#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "1q/electro_optical_sensor/session/EosOutputDebugView.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electro_optical_sensor/session/EosSessionFactory.h"
#include "1q/electro_optical_sensor/session/EosTraceSession.h"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"
#include "1q/electronic_surveillance_radar/config/electronic_surveillance_radar_config.hpp"
#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentConfig.h"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentRuntimeConfigPatch.h"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentSceneBuilder.h"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
#include "1q/electronic_surveillance_radar/environment/electronic_surveillance_radar_environment.hpp"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInputBuilder.h"
#include "1q/electronic_surveillance_radar/session/EsrEmitterLifecycleRecorder.h"
#include "1q/electronic_surveillance_radar/session/EsrEnvironmentInput.h"
#include "1q/electronic_surveillance_radar/session/EsrEnvironmentInputPatch.h"
#include "1q/electronic_surveillance_radar/session/EsrEnvironmentInputState.h"
#include "1q/electronic_surveillance_radar/session/EsrExternalInputAdapter.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrOutputDebugView.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/electronic_surveillance_radar/session/EsrSessionFactory.h"
#include "1q/electronic_surveillance_radar/session/EsrTraceSession.h"
#include "1q/foundation/atmospheric_types.h"
#include "1q/foundation/pose_types.h"
#include "1q/foundation/scan_schedule_types.h"
#include "1q/sar/config/SarEnvironmentConfig.h"
#include "1q/sar/config/SarHardwareConfig.h"
#include "1q/sar/config/SarMissionConfig.h"
#include "1q/sar/config/SarPolicyConfig.h"
#include "1q/sar/config/SarRuntimeConfigPatch.h"
#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/config/sar_config.hpp"
#include "1q/sar/sar.hpp"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"
#include "1q/sar/session/SarProductDebugView.h"
#include "1q/sar/session/SarProductLifecycleRecorder.h"
#include "1q/sar/session/SarReplaySession.h"
#include "1q/sar/session/SarSession.h"
#include "1q/sar/session/SarSessionFactory.h"
#include "1q/sar/session/SarTraceSession.h"
#include "1q/trace/TraceSink.h"

using ArSession = airborne_radar::session::RadarSession;
using ArConfig = airborne_radar::config::RadarSessionConfig;
static_assert(!std::is_constructible<ArSession, const ArConfig&>::value,
              "RadarSession direct construction must be disabled");
static_assert(std::is_same<ArSession, decltype(airborne_radar::session::RadarSessionFactory::Create(
                                          std::declval<const ArConfig&>()))>::value,
              "RadarSessionFactory::Create must return RadarSession");
static_assert(
    std::is_same<
        ArSession,
        decltype(airborne_radar::session::RadarSessionFactory::CreateWithDecisionEngine(
            std::declval<const ArConfig&>(),
            std::declval<airborne_radar::extension::ITacticalDecisionEngine&>()))>::value,
    "RadarSessionFactory::CreateWithDecisionEngine must return RadarSession");

static_assert(
    !std::is_constructible<electronic_surveillance_radar::session::EsrSession,
                           electronic_surveillance_radar::config::EsrSessionConfig>::value,
    "EsrSession direct construction must be disabled");

static_assert(!std::is_constructible<electro_optical_sensor::session::EosSession,
                                     electro_optical_sensor::config::EosSessionConfig>::value,
              "EosSession direct construction must be disabled");
static_assert(
    std::is_same<
        electro_optical_sensor::session::EosSession,
        decltype(electro_optical_sensor::session::EosSessionFactory::Create(
            std::declval<const electro_optical_sensor::config::EosSessionConfig&>()))>::value,
    "EosSessionFactory::Create must return EosSession");

static_assert(
    !std::is_constructible<sar::session::SarSession, sar::config::SarSessionConfig>::value,
    "SarSession direct construction must be disabled");
static_assert(std::is_same<sar::session::SarSession,
                           decltype(sar::session::SarSessionFactory::Create(
                               std::declval<const sar::config::SarSessionConfig&>()))>::value,
              "SarSessionFactory::Create must return SarSession");

namespace airborne_radar {
namespace {

TEST(PublicHeadersSmokeTest, StablePublicSurfaceSupportsMinimalUsage) {
  oneq::coordinate::LlaPositionDegM origin_lla;
  origin_lla.latitude_deg = 0.0;
  origin_lla.longitude_deg = 0.0;
  origin_lla.altitude_m = 0.0;
  oneq::coordinate::EcefPositionM origin_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(origin_lla, &origin_ecef));

  config::RadarSessionConfig session_config = config::RadarSessionConfigBuilder().Build();
  session_config.environment.scenario_config.atmospheric_physics.enable_physical_model = true;

  session::RadarCycleInput input;
  input.dt_sec = 1.0f;
  session::RadarEnvironmentInputState environment_state(input.environment);
  session::RadarEnvironmentInputPatch environment_patch;
  environment_patch.has_jammer_sources = true;
  environment_patch.jammer_sources.push_back(environment::JammerEmitterState{});
  input.environment = environment_state.Update(environment_patch).Snapshot();
  const std::vector<session::ValidationIssue> issues = session::ValidateRadarCycleInput(input);

  EXPECT_FALSE(session::HasValidationError(issues));

  session::RadarSession session = session::RadarSessionFactory::Create(session_config);
  session::RadarTraceSession trace_session(session_config,
                                           session::RadarTraceSessionOptions{nullptr, false});
  const config::RadarRuntimeConfigPatch runtime_patch =
      config::RadarRuntimeConfigBuilder()
          .WithWorkMode(model::RadarWorkMode::kTas)
          .WithCommandedBeamwidthEnabled(true)
          .Build();
  session.ApplyRuntimeConfig(runtime_patch);
  const session::RadarCycleResult result = session.StepWithResult(input);
  const session::RadarCycleResult trace_result = trace_session.StepWithResult(input);
  const std::size_t confirmed_tracks =
      session::CountTracksByStatus(result.track_output_frame, model::TrackStatus::kConfirmed);

  EXPECT_GE(confirmed_tracks, 0U);
  EXPECT_GE(result.association_quality_metrics.detection_count, 0U);
  EXPECT_GE(trace_result.association_quality_metrics.detection_count, 0U);
}

TEST(PublicHeadersSmokeTest, FourDomainHeadersDefineIndependentConfigTypes) {
  config::RadarHardwareConfig hardware{};
  hardware.detection.pulse_count = 32;
  EXPECT_EQ(hardware.detection.pulse_count, 32);

  model::AzimuthElevationDeg scan_center;
  scan_center.az_deg = 45.0f;
  scan_center.el_deg = -5.0f;
  config::RadarMissionConfig mission{};
  mission.orientation.scan_center_deg = scan_center;
  EXPECT_FLOAT_EQ(mission.orientation.scan_center_deg.az_deg, 45.0f);

  config::RadarPolicyConfig policy{};
  policy.lifecycle.confirm_hits = 2U;
  policy.tracking.kalman_update_backend = config::KalmanUpdateBackend::kUdKf;
  EXPECT_EQ(policy.lifecycle.confirm_hits, 2U);
  EXPECT_EQ(policy.tracking.kalman_update_backend, config::KalmanUpdateBackend::kUdKf);

  config::RadarEnvironmentConfig env{};
  env.scenario_config.atmospheric_physics.enable_physical_model = true;
  EXPECT_TRUE(env.scenario_config.atmospheric_physics.enable_physical_model);

  config::RadarSessionConfig session_cfg;
  session_cfg.hardware = hardware;
  session_cfg.mission = mission;
  session_cfg.policy = policy;
  session_cfg.environment = env;
  session_cfg.environment.jamming_sensitivity_profile = environment::JammingSensitivityProfile::kStrict;
  EXPECT_EQ(session_cfg.hardware.detection.pulse_count, 32);
  EXPECT_FLOAT_EQ(session_cfg.mission.orientation.scan_center_deg.az_deg, 45.0f);
  EXPECT_EQ(session_cfg.policy.lifecycle.confirm_hits, 2U);
  EXPECT_EQ(session_cfg.environment.jamming_sensitivity_profile,
            environment::JammingSensitivityProfile::kStrict);
}

TEST(PublicHeadersSmokeTest, RadarSessionBuilderCanConfigureLeafAndDomainFields) {
  model::AzimuthElevationDeg scan_center;
  scan_center.az_deg = -12.0f;
  scan_center.el_deg = 6.0f;

  const config::RadarSessionConfig config =
      config::RadarSessionConfigBuilder()
          .Detection()
          .EnablePhysicsDetection(true)
          .WithDetectionIntentProfile(config::profiles::DetectionIntentProfile::kDetectionPriority)
          .WithHardwareProfile(config::profiles::RadarHardwareProfile::kLongRangeHighPower)
          .WithAntennaPatternProfile(config::profiles::AntennaPatternProfile::kLowSidelobe)
          .End()
          .Mission()
          .WithScanCenterDeg(scan_center)
          .End()
          .Lifecycle()
          .WithLifecyclePolicyProfile(config::profiles::LifecyclePolicyProfile::kFastConfirm)
          .End()
          .Environment()
          .WithJammingSensitivityProfile(environment::JammingSensitivityProfile::kRelaxed)
          .End()
          .Build();
  EXPECT_TRUE(config.hardware.detection.enable_physics_detection);
  EXPECT_EQ(config.hardware.detection.pulse_count, 16);
  EXPECT_FLOAT_EQ(config.hardware.detection.transmitter.peak_power_w, 5.0e6f);
  EXPECT_FLOAT_EQ(config.hardware.detection.antenna.pattern.max_sidelobe_level_db, -30.0f);
  EXPECT_FLOAT_EQ(config.mission.orientation.scan_center_deg.az_deg, -12.0f);
  EXPECT_EQ(config.policy.lifecycle.confirm_hits, 1U);
  EXPECT_EQ(config.environment.jamming_sensitivity_profile, environment::JammingSensitivityProfile::kRelaxed);

  environment::EnvironmentScenarioConfig scenario;
  scenario.atmospheric_physics.enable_physical_model = true;
  environment::EnvironmentDefaultConfig env;
  env.scenario_config = scenario;
  EXPECT_TRUE(env.scenario_config.atmospheric_physics.enable_physical_model);
}

}  // namespace
}  // namespace airborne_radar

namespace electronic_surveillance_radar {
namespace {

TEST(PublicHeadersSmokeTest, EsrPublicSurfaceSupportsMinimalUsage) {
  const oneq::foundation::ScanStartPosition shared_start =
      oneq::foundation::ScanStartPosition::kLeftTop;
  EXPECT_EQ(static_cast<int>(shared_start), 0);

  config::EsrSessionConfig session_config =
      config::EsrSessionConfigBuilder()
          .Mission()
          .WithScanRateHz(1.0f)
          .End()
          .Detection()
          .WithMinDetectSnrDb(6.0f)
          .End()
          .Build();
  session_config.mission.scan.use_explicit_scan_bounds = true;
  session_config.mission.scan.scan_start_az_deg = -60.0f;
  session_config.mission.scan.scan_end_az_deg = 60.0f;
  session_config.mission.scan.scan_start_el_deg = -20.0f;
  session_config.mission.scan.scan_end_el_deg = 20.0f;
  session_config.hardware.beam_az_width_deg = 120.0f;
  session_config.hardware.beam_el_width_deg = 40.0f;

  session::EsrCycleInput input;
  input.cycle_index = 4U;
  input.dt_sec = 1.0f;
  session::EsrEnvironmentInputState environment_state(input.environment);
  session::EsrEnvironmentInputPatch environment_patch;
  environment_patch.has_spectrum_occupancy_ratio = true;
  environment_patch.spectrum_occupancy_ratio = 0.25f;
  input.environment = environment_state.Update(environment_patch).Snapshot();
  session::EsrSceneEmitter emitter;
  emitter.emitter_id = 1001U;
  emitter.pose.position_m.x = 1200.0f;
  emitter.carrier_hz = 10.0e9;
  emitter.bandwidth_hz = 2.0e6;
  emitter.tx_power_w = 5.0e7;
  emitter.pulse_width_s = 1.0e-6;
  emitter.pri_s = 1.0e-4;
  input.scene.push_back(emitter);

  oneq::coordinate::LocalFrameReference esr_reference;
  esr_reference.origin_lla.latitude_deg = 0.0;
  esr_reference.origin_lla.longitude_deg = 0.0;
  esr_reference.origin_lla.altitude_m = 0.0;
  oneq::coordinate::EcefPositionM esr_origin_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(esr_reference.origin_lla, &esr_origin_ecef));
  session::EsrExternalPoseInput esr_pose_input;
  esr_pose_input.platform_position_ecef_m = esr_origin_ecef;
  oneq::foundation::PoseState esr_pose;
  ASSERT_TRUE(
      session::TryMakeEsrPoseFromExternalKinematics(esr_pose_input, esr_reference, &esr_pose));

  const session::ValidationIssueList issues = session::ValidateEsrCycleInput(input);
  EXPECT_FALSE(session::HasValidationError(issues));

  auto session = session::EsrSessionFactory::Create(session_config);
  const config::EsrRuntimeConfigPatch runtime_patch =
      config::EsrRuntimeConfigBuilder().WithWorkMode(config::EsrWorkMode::kRwr).Build();
  session.ApplyRuntimeConfig(runtime_patch);
  const session::EsrCycleResult result = session.StepWithResult(input);
  session::EsrTraceSession trace_session(session_config, session::EsrTraceSessionOptions{});
  const session::EsrCycleResult trace_result = trace_session.StepWithResult(input);

  EXPECT_GE(result.output_frame.observation_output.observations.size(), 0U);
  EXPECT_GE(result.output_frame.emitter_output.hypotheses.size(), 0U);
  EXPECT_GE(result.output_frame.truth_evaluation_output.associations.size(), 0U);
  EXPECT_GE(trace_result.output_frame.truth_evaluation_output.associations.size(), 0U);
}

}  // namespace
}  // namespace electronic_surveillance_radar

namespace electro_optical_sensor {
namespace {

TEST(PublicHeadersSmokeTest, EosPublicSurfaceSupportsMinimalUsage) {
  config::EosSessionConfig session_config =
      config::EosSessionConfigBuilder()
          .Mission()
          .WithWorkMode(config::EosWorkMode::kFused)
          .End()
          .Build();
  session_config.policy.detection.minimum_snr_db = 4.5f;
  session_config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  session_config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;
  session_config.mission.scan_start_az_deg = -20.0f;
  session_config.mission.scan_end_az_deg = 20.0f;

  ::electro_optical_sensor::session::EosCycleInput input;
  input.cycle_index = 2U;
  input.dt_sec = 1.0f;
  session::EosEnvironmentInputState environment_state(input.environment);
  session::EosEnvironmentInputPatch environment_patch;
  environment_patch.has_day_night_type = true;
  environment_patch.day_night_type = ::electro_optical_sensor::session::DayNightType::kDay;
  input.environment = environment_state.Update(environment_patch).Snapshot();
  session::EosSceneTarget target;
  target.target_id = 7U;
  target.range_m = 1500.0f;
  target.azimuth_deg = 0.0f;
  target.elevation_deg = 0.0f;
  target.appearance.apparent_temperature_k = 320.0f;
  target.appearance.emissivity = 0.9f;
  target.appearance.reflectance = 0.4f;
  target.appearance.projected_area_m2 = 2.0f;
  input.scene.push_back(target);

  oneq::coordinate::LocalFrameReference eos_reference;
  eos_reference.origin_lla.latitude_deg = 0.0;
  eos_reference.origin_lla.longitude_deg = 0.0;
  eos_reference.origin_lla.altitude_m = 0.0;
  oneq::coordinate::EcefPositionM eos_origin_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(eos_reference.origin_lla, &eos_origin_ecef));
  session::EosExternalPoseInput eos_pose_input;
  eos_pose_input.platform_position_ecef_m = eos_origin_ecef;
  oneq::foundation::PoseState eos_pose;
  ASSERT_TRUE(
      session::TryMakeEosPoseFromExternalKinematics(eos_pose_input, eos_reference, &eos_pose));

  const session::ValidationIssueList issues = session::ValidateEosCycleInput(input);
  EXPECT_FALSE(session::HasValidationError(issues));

  foundation::radiative_transfer::RadiativeTransferInputs transfer_inputs;
  transfer_inputs.model =
      foundation::radiative_transfer::RadiativeTransferModel::kAdaptivePathRadiance;
  transfer_inputs.base_transmittance = 0.8f;
  transfer_inputs.cloud_coverage_ratio = 0.2f;
  transfer_inputs.path_length_m = 1500.0f;
  const foundation::radiative_transfer::RadiativeTransferResult transfer_result =
      foundation::radiative_transfer::EvaluateRadiativeTransfer(transfer_inputs);
  EXPECT_GT(transfer_result.transmittance, 0.0f);

  session::EosSession session = session::EosSessionFactory::Create(session_config);
  const config::EosRuntimeConfigPatch runtime_patch =
      config::EosRuntimeConfigBuilder().WithFrameRateHz(15.0f).Build();
  session.ApplyRuntimeConfig(runtime_patch);
  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);
  session::EosTraceSession trace_session(session_config, session::EosTraceSessionOptions{});
  const ::electro_optical_sensor::session::EosCycleResult trace_result =
      trace_session.StepWithResult(input);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.reused_previous_output);
  EXPECT_TRUE(trace_result.executed_this_cycle);
  EXPECT_FALSE(trace_result.reused_previous_output);
  EXPECT_GE(result.output_frame.detections.size(), 0U);
  EXPECT_GE(trace_result.output_frame.detections.size(), 0U);
}

}  // namespace
}  // namespace electro_optical_sensor

namespace sar {
namespace {

TEST(PublicHeadersSmokeTest, SarPublicSurfaceSupportsMinimalUsage) {
  config::SarSessionConfig session_config;
  session_config.hardware.carrier_frequency_hz = 1.0e9;
  session_config.hardware.bandwidth_hz = 25.0e6;
  session_config.hardware.pulse_width_s = 0.16e-6;
  session_config.hardware.pulse_repetition_frequency_hz = 20.0;
  session_config.hardware.sample_rate_hz = 100.0e6;
  session_config.mission.nominal_slant_range_m = 29.9792458;
  session_config.mission.platform_speed_mps = 2.0;
  session_config.mission.range_sample_count = 64U;
  session_config.mission.azimuth_pulse_count = 9U;
  session_config.policy.enable_l1_rda_imaging = true;
  EXPECT_FALSE(session_config.policy.enable_l2_motion_compensation);
  EXPECT_FALSE(session_config.policy.enable_l3_bp_imaging);
  EXPECT_DOUBLE_EQ(session_config.mission.l2_velocity_error_stddev_y_mps, 0.0);
  config::SarWaypointConfig waypoint;
  waypoint.time_from_session_start_s = 0.0;
  session_config.mission.l3_waypoints.push_back(waypoint);
  EXPECT_EQ(session_config.mission.l3_waypoints.size(), 1U);
  EXPECT_EQ(session_config.mission.range_sample_count, 64U);

  session::SarCycleInput input;
  input.cycle_index = 8U;
  input.dt_sec = 0.5;
  input.platform.altitude_m = 0.0;
  session::SarPointTarget target;
  target.latitude_deg = 29.9792458 / 6378137.0 * 180.0 / 3.14159265358979323846;
  target.longitude_deg = 0.0;
  target.radar_cross_section_dbsm = 80.0;
  input.point_targets.push_back(target);
  session::SarRawIqFrame raw_iq;
  raw_iq.pulse_count = 1U;
  raw_iq.samples_per_pulse = 1U;
  raw_iq.i_values.push_back(1.0);
  raw_iq.q_values.push_back(0.0);
  session::SarRawIqFrame::PulseState pulse_state;
  pulse_state.pulse_id = 0U;
  raw_iq.pulse_states.push_back(pulse_state);
  raw_iq.ideal_pulse_states.push_back(pulse_state);
  EXPECT_EQ(raw_iq.i_values.size(), 1U);
  EXPECT_EQ(raw_iq.pulse_states.size(), 1U);
  EXPECT_EQ(raw_iq.ideal_pulse_states.size(), 1U);

  session::SarSession session = session::SarSessionFactory::Create(session_config);
  config::SarRuntimeConfigPatch patch;
  patch.has_retain_raw_phase_history = true;
  patch.retain_raw_phase_history = true;
  EXPECT_TRUE(session.TryApplyRuntimeConfig(patch));

  const session::SarCycleResult result = session.StepWithResult(input);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.reused_previous_output);
  EXPECT_EQ(result.output_frame.range_sample_count, 64U);
  EXPECT_TRUE(result.output_frame.has_raw_echo);
  EXPECT_TRUE(result.output_frame.has_range_compressed_echo);
  EXPECT_TRUE(result.output_frame.has_l1_image);
  EXPECT_FALSE(result.output_frame.has_l3_bp_image);
  EXPECT_EQ(result.focused_image.source, session::SarFocusedImageSource::kL1Rda);
  EXPECT_EQ(result.focused_image.row_count, 9U);
  EXPECT_EQ(result.focused_image.column_count, 64U);
  EXPECT_EQ(result.focused_image.real_values.size(), 9U * 64U);
  EXPECT_EQ(result.focused_image.imaginary_values.size(), 9U * 64U);
  EXPECT_FALSE(result.focused_image.is_placeholder);

  session::SarTraceSession trace_session(session::SarSessionFactory::Create(session_config));
  const session::SarCycleResult trace_result = trace_session.StepWithResult(input);
  EXPECT_TRUE(trace_result.executed_this_cycle);

  session::SarReplaySessionResult replay_result;
  EXPECT_FALSE(replay_result.ok);
}

}  // namespace
}  // namespace sar
