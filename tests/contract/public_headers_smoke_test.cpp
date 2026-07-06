/**
 * @file public_headers_smoke_test.cpp
 * @brief 验证稳定公共头集合可被统一包含并完成最小用法编译。
 */

#include <gtest/gtest.h>

#include <memory>
#include <type_traits>
#include <utility>

#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/airborne_radar/config/ArEnvironmentConfig.h"
#include "1q/airborne_radar/config/ArHardwareConfig.h"
#include "1q/airborne_radar/config/ArMissionConfig.h"
#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "1q/airborne_radar/config/ArPolicyConfig.h"
#include "1q/airborne_radar/config/ArRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/ArRuntimeConfigPatch.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/config/ArSessionConfigBuilder.h"
#include "1q/airborne_radar/config/JammingSemantics.h"
#include "1q/airborne_radar/config/airborne_radar_config.hpp"
#include "1q/airborne_radar/session/ArCommand.h"
#include "1q/airborne_radar/session/ArControlProfile.h"
#include "1q/airborne_radar/session/ArCycleInput.h"
#include "1q/airborne_radar/session/ArCycleInputAdapter.h"
#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/airborne_radar/session/ArEnvironmentInput.h"
#include "1q/airborne_radar/session/ArInputValidation.h"
#include "1q/airborne_radar/session/ArOutputTypes.h"
#include "1q/airborne_radar/session/ArSession.h"
#include "1q/airborne_radar/session/ArTraceSession.h"
#include "1q/airborne_radar/session/ArTrackLifecycleRecorder.h"
#include "1q/airborne_radar/session/ArTrackOutputDebugView.h"
#include "1q/airborne_radar/session/ControlDirective.h"
#include "1q/airborne_radar/session/DecisionInputFrame.h"
#include "1q/airborne_radar/session/DecisionSourceInfo.h"
#include "1q/airborne_radar/session/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/session/TrackStateSnapshot.h"
#include "1q/api.hpp"
#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/types.h"
#include "1q/coordinate/velocity_transform.h"
#include "1q/electro_optical_sensor/config/EosEnvironmentConfig.h"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/config/electro_optical_sensor_config.hpp"
#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleInputAdapter.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosDetectionLifecycleRecorder.h"
#include "1q/electro_optical_sensor/session/EosEnvironmentInput.h"
#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "1q/electro_optical_sensor/session/EosOutputDebugView.h"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electro_optical_sensor/session/EosTraceSession.h"
#include "1q/electronic_surveillance_radar/config/EsrEnvironmentConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"
#include "1q/electronic_surveillance_radar/config/electronic_surveillance_radar_config.hpp"
#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInputAdapter.h"
#include "1q/electronic_surveillance_radar/session/EsrEmitterLifecycleRecorder.h"
#include "1q/electronic_surveillance_radar/session/EsrEnvironmentInput.h"
#include "1q/electronic_surveillance_radar/session/EsrExternalInputAdapter.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrOutputDebugView.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/electronic_surveillance_radar/session/EsrTraceSession.h"
#include "1q/environment/AtmosphericTypes.h"
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
#include "1q/sar/session/SarTraceSession.h"
#include "1q/sbirs_sensor/config/SbirsRuntimeConfigBuilder.h"
#include "1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfig.h"
#include "1q/sbirs_sensor/config/SbirsSessionConfigBuilder.h"
#include "1q/sbirs_sensor/config/sbirs_sensor_config.hpp"
#include "1q/sbirs_sensor/sbirs_sensor.hpp"
#include "1q/sbirs_sensor/session/SbirsCycleInput.h"
#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsCycleResult.h"
#include "1q/sbirs_sensor/session/SbirsEnvironmentInput.h"
#include "1q/sbirs_sensor/session/SbirsExternalInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsInputValidation.h"
#include "1q/sbirs_sensor/session/SbirsOutputTypes.h"
#include "1q/sbirs_sensor/session/SbirsSession.h"
#include "1q/trace/TraceSink.h"

using ArSession = airborne_radar::session::ArSession;
using ArConfig = airborne_radar::config::ArSessionConfig;
static_assert(!std::is_constructible<ArSession, const ArConfig&>::value,
              "ArSession direct construction must be disabled");
static_assert(std::is_same<ArSession, decltype(airborne_radar::session::ArSession::Create(
                                          std::declval<const ArConfig&>()))>::value,
              "ArSession::Create must return ArSession");
static_assert(
    std::is_same<ArSession,
                 decltype(airborne_radar::session::ArSession::CreateWithDecisionEngine(
                     std::declval<const ArConfig&>(),
                     std::declval<airborne_radar::session::ITacticalDecisionEngine&>()))>::value,
    "ArSession::CreateWithDecisionEngine must return ArSession");

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
        decltype(electro_optical_sensor::session::EosSession::Create(
            std::declval<const electro_optical_sensor::config::EosSessionConfig&>()))>::value,
    "EosSession::Create must return EosSession");

static_assert(
    !std::is_constructible<sar::session::SarSession, sar::config::SarSessionConfig>::value,
    "SarSession direct construction must be disabled");
static_assert(std::is_same<sar::session::SarSession,
                           decltype(sar::session::SarSession::Create(
                               std::declval<const sar::config::SarSessionConfig&>()))>::value,
              "SarSession::Create must return SarSession");

static_assert(!std::is_constructible<sbirs_sensor::session::SbirsSession,
                                     sbirs_sensor::config::SbirsSessionConfig>::value,
              "SbirsSession direct construction must be disabled");
static_assert(
    std::is_same<sbirs_sensor::session::SbirsSession,
                 decltype(sbirs_sensor::session::SbirsSession::Create(
                     std::declval<const sbirs_sensor::config::SbirsSessionConfig&>()))>::value,
    "SbirsSession::Create must return SbirsSession");

namespace airborne_radar {
namespace {

TEST(PublicHeadersSmokeTest, StablePublicSurfaceSupportsMinimalUsage) {
  oneq::coordinate::LlaPositionDegM origin_lla;
  origin_lla.latitude_deg = 0.0;
  origin_lla.longitude_deg = 0.0;
  origin_lla.altitude_m = 0.0;
  oneq::coordinate::EcefPositionM origin_ecef;
  ASSERT_TRUE(oneq::coordinate::TryLlaToEcef(origin_lla, &origin_ecef));

  config::ArSessionConfig session_config = config::ArSessionConfigBuilder().Build();
  session_config.environment.scenario_config.atmospheric_physics.enable_physical_model = true;

  session::ArCycleInput input;
  input.dt_sec = 1.0f;
  session::ArEnvironmentInputState environment_state(input.environment);
  session::ArEnvironmentInputPatch environment_patch;
  environment_patch.has_jammer_sources = true;
  environment_patch.jammer_sources.push_back(config::JammerEmitterState{});
  input.environment = environment_state.Update(environment_patch).Snapshot();
  input.has_environment = true;
  const std::vector<session::ValidationIssue> issues = session::ValidateArCycleInput(input);

  EXPECT_FALSE(session::HasValidationError(issues));

  session::ArSession session = session::ArSession::Create(session_config);
  session::ArTraceSession trace_session(session_config,
                                        session::ArTraceSessionOptions{nullptr, false});
  const config::ArRuntimeConfigPatch runtime_patch = config::ArRuntimeConfigBuilder()
                                                         .WithWorkMode(config::ArWorkMode::kTas)
                                                         .WithCommandedBeamwidthEnabled(true)
                                                         .Build();
  session.ApplyRuntimeConfig(runtime_patch);
  const session::ArCycleResult result = session.StepWithResult(input);
  const session::ArCycleResult trace_result = trace_session.StepWithResult(input);
  const std::size_t confirmed_tracks =
      session::CountTracksByStatus(result.track_output_frame, session::TrackStatus::kConfirmed);

  EXPECT_GE(confirmed_tracks, 0U);
  EXPECT_GE(result.association_quality_metrics.detection_count, 0U);
  EXPECT_GE(trace_result.association_quality_metrics.detection_count, 0U);
}

TEST(PublicHeadersSmokeTest, SbirsPublicSurfaceSupportsMinimalUsage) {
  sbirs_sensor::config::SbirsSessionConfig config =
      sbirs_sensor::config::SbirsSessionConfigBuilder().Build();
  sbirs_sensor::session::SbirsSession session = sbirs_sensor::session::SbirsSession::Create(config);

  sbirs_sensor::session::SbirsVector3M satellite;
  satellite.x = 7000000.0;
  satellite.y = 0.0;
  satellite.z = 0.0;
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.position_ecef_m.x = 8000000.0;
  target.position_ecef_m.y = 0.0;
  target.position_ecef_m.z = 0.0;
  target.temperature_k = 1800.0f;
  target.projected_area_m2 = 100.0f;

  sbirs_sensor::session::SbirsCycleInput input = sbirs_sensor::session::SbirsCycleInputBuilder()
                                                     .WithCycleIndex(1U)
                                                     .WithDeltaTimeSec(1.0f)
                                                     .WithSatellitePosition(satellite)
                                                     .AddTarget(target)
                                                     .Build();
  const sbirs_sensor::session::ValidationIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input);
  EXPECT_FALSE(sbirs_sensor::session::HasValidationError(issues));

  const sbirs_sensor::config::SbirsRuntimeConfigPatch patch =
      sbirs_sensor::config::SbirsRuntimeConfigBuilder()
          .WithWorkMode(sbirs_sensor::config::SbirsWorkMode::kSearchAndStare)
          .Build();
  EXPECT_TRUE(session.TryApplyRuntimeConfig(patch));
  const sbirs_sensor::session::SbirsCycleResult result = session.StepWithResult(input);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_TRUE(sbirs_sensor::session::SbirsOutputFrameContainsOnlyNativeFields(result.output_frame));
}

TEST(PublicHeadersSmokeTest, FourDomainHeadersDefineIndependentConfigTypes) {
  config::ArHardwareConfig hardware{};
  hardware.pulse_count = 32;
  EXPECT_EQ(hardware.pulse_count, 32);

  config::AzimuthElevationDeg scan_center;
  scan_center.az_deg = 45.0f;
  scan_center.el_deg = -5.0f;
  config::ArMissionConfig mission{};
  mission.orientation.scan_center_deg = scan_center;
  EXPECT_FLOAT_EQ(mission.orientation.scan_center_deg.az_deg, 45.0f);

  config::ArPolicyConfig policy{};
  policy.lifecycle.confirm_hits = 2U;
  policy.tracking.kalman_update_backend = config::KalmanUpdateBackend::kUdKf;
  EXPECT_EQ(policy.lifecycle.confirm_hits, 2U);
  EXPECT_EQ(policy.tracking.kalman_update_backend, config::KalmanUpdateBackend::kUdKf);

  config::ArEnvironmentConfig env{};
  env.scenario_config.atmospheric_physics.enable_physical_model = true;
  EXPECT_TRUE(env.scenario_config.atmospheric_physics.enable_physical_model);

  config::ArSessionConfig session_cfg;
  session_cfg.hardware = hardware;
  session_cfg.mission = mission;
  session_cfg.policy = policy;
  session_cfg.environment = env;
  session_cfg.environment.jamming_sensitivity_profile = config::JammingSensitivityProfile::kStrict;
  EXPECT_EQ(session_cfg.hardware.pulse_count, 32);
  EXPECT_FLOAT_EQ(session_cfg.mission.orientation.scan_center_deg.az_deg, 45.0f);
  EXPECT_EQ(session_cfg.policy.lifecycle.confirm_hits, 2U);
  EXPECT_EQ(session_cfg.environment.jamming_sensitivity_profile,
            config::JammingSensitivityProfile::kStrict);
}

TEST(PublicHeadersSmokeTest, RadarSessionBuilderCanConfigureLeafAndDomainFields) {
  config::AzimuthElevationDeg scan_center;
  scan_center.az_deg = -12.0f;
  scan_center.el_deg = 6.0f;

  config::ArSessionConfig config =
      config::ArSessionConfigBuilder()
          .Detection()
          .EnablePhysicsDetection(true)
          .WithDetectionIntentProfile(config::profiles::DetectionIntentProfile::kDetectionPriority)
          .WithHardwareProfile(config::profiles::ArHardwareProfile::kLongRangeHighPower)
          .WithAntennaPatternProfile(config::profiles::AntennaPatternProfile::kLowSidelobe)
          .End()
          .Lifecycle()
          .WithLifecyclePolicyProfile(config::profiles::LifecyclePolicyProfile::kFastConfirm)
          .End()
          .Environment()
          .WithJammingSensitivityProfile(config::JammingSensitivityProfile::kRelaxed)
          .End()
          .Build();
  config.mission.orientation.scan_center_deg = scan_center;
  EXPECT_TRUE(config.hardware.enable_physics_detection);
  EXPECT_EQ(config.hardware.pulse_count, 16);
  EXPECT_FLOAT_EQ(config.hardware.transmitter.peak_power_w, 5.0e6f);
  EXPECT_FLOAT_EQ(config.hardware.antenna.pattern.max_sidelobe_level_db, -30.0f);
  EXPECT_FLOAT_EQ(config.mission.orientation.scan_center_deg.az_deg, -12.0f);
  EXPECT_EQ(config.policy.lifecycle.confirm_hits, 1U);
  EXPECT_EQ(config.environment.jamming_sensitivity_profile,
            config::JammingSensitivityProfile::kRelaxed);

  config::EnvironmentScenarioConfig scenario;
  scenario.atmospheric_physics.enable_physical_model = true;
  config::ArEnvironmentConfig env;
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
          .WithMissionProfile(config::EsrMissionProfile::kElectronicOrderOfBattle)
          .End()
          .Detection()
          .WithSensitivityProfile(config::EsrSensitivityProfile::kStandard)
          .End()
          .Build();
  session_config.mission.scan.scan_rate_hz = 1.0f;
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

  auto session = session::EsrSession::Create(session_config);
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
          .WithMissionProfile(config::EosMissionProfile::kWideAreaSearch)
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

  // RadiativeTransferModel 枚举通过 EosEnvironmentConfig.h 公开
  EXPECT_NE(static_cast<int>(config::RadiativeTransferModel::kDerivedBeerLambert), -1);

  session::EosSession session = session::EosSession::Create(session_config);
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
  input.dt_sec = 0.5f;
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

  session::SarSession session = session::SarSession::Create(session_config);
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

  session::SarTraceSession trace_session(session::SarSession::Create(session_config));
  const session::SarCycleResult trace_result = trace_session.StepWithResult(input);
  EXPECT_TRUE(trace_result.executed_this_cycle);

  session::SarReplaySessionResult replay_result;
  EXPECT_FALSE(replay_result.ok);
}

}  // namespace
}  // namespace sar
