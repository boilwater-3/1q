/**
 * @file public_headers_smoke_test.cpp
 * @brief 验证稳定公共头集合可被统一包含并完成最小用法编译。
 */

#include <gtest/gtest.h>

#include <memory>
#include <type_traits>
#include <utility>

#include "1q/airborne_radar/extension/control/ControlDirective.h"
#include "1q/airborne_radar/extension/control/RadarCommand.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/model/DecisionInputFrame.h"
#include "1q/airborne_radar/model/DecisionSourceInfo.h"
#include "1q/airborne_radar/model/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/model/TargetCategory.h"
#include "1q/airborne_radar/model/TargetFeature.h"
#include "1q/airborne_radar/output/TrackOutputFrame.h"
#include "1q/airborne_radar/output/TrackOutputQueries.h"
#include "1q/airborne_radar/model/JammingSemantics.h"
#include "1q/airborne_radar/model/TargetFeatureUtils.h"
#include "1q/airborne_radar/config/AntennaPatternConfig.h"
#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/airborne_radar/config/RadarSessionConfigPresets.h"
#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/config/SignalBeamControlConfig.h"
#include "1q/airborne_radar/config/SignalDetectionConfig.h"
#include "1q/airborne_radar/config/SignalLifecycleConfig.h"
#include "1q/airborne_radar/config/SignalPipelineConfig.h"
#include "1q/airborne_radar/config/SignalTrackingConfig.h"
#include "1q/airborne_radar/config/airborne_radar_config.hpp"
#include "1q/airborne_radar/extension/IRadarContext.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarInputValidation.h"
#include "1q/airborne_radar/extension/IRadarOutputReader.h"
#include "1q/airborne_radar/extension/RadarController.h"
#include "1q/airborne_radar/session/RadarCycleResult.h"
#include "1q/airborne_radar/session/RadarSessionFactory.h"
#include "1q/airborne_radar/session/RadarSession.h"
#include "1q/airborne_radar/extension/ControlReducerTypes.h"
#include "1q/airborne_radar/extension/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/extension/airborne_radar_extension.hpp"
#include "1q/airborne_radar/environment/EnvironmentConfig.h"
#include "1q/airborne_radar/environment/EnvironmentDefaultConfigBuilder.h"
#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"
#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/extension/IEnvironmentService.h"
#include "1q/airborne_radar/extension/ISignalPipeline.h"
#include "1q/airborne_radar/extension/SignalPipelineResultTypes.h"
#include "1q/airborne_radar/airborne_radar.hpp"
#include "1q/api.hpp"
#include "1q/common/coordinate_transform.h"
#include "1q/common/pose_types.h"
#include "1q/common/scan_schedule_types.h"
#include "1q/common/trace/TraceSink.h"
#include "1q/electro_optical_sensor/output/EosOutputFrame.h"
#include "1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h"
#include "1q/electro_optical_sensor/model/EosCycleInput.h"
#include "1q/electro_optical_sensor/utils/EosCoordinateUtils.h"
#include "1q/electro_optical_sensor/model/EosInputValidation.h"
#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/model/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electro_optical_sensor/environment/EosEnvironmentTypes.h"
#include "1q/electro_optical_sensor/extension/IEosEnvironmentService.h"
#include "1q/electro_optical_sensor/extension/EosPipelineTypes.h"
#include "1q/electro_optical_sensor/extension/IEosPipeline.h"
#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/extension/EosPipelineTypes.h"
#include "1q/electro_optical_sensor/extension/IEosEnvironmentService.h"
#include "1q/electro_optical_sensor/extension/IEosPipeline.h"
#include "1q/electro_optical_sensor/extension/electro_optical_sensor_extension.hpp"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/config/electro_optical_sensor_config.hpp"
#include "1q/electro_optical_sensor/session/EosTraceSession.h"
#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"
#include "1q/electronic_surveillance_radar/model/EmitterTruthState.h"
#include "1q/electronic_surveillance_radar/model/EsrCoordinateUtils.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"
#include "1q/electronic_surveillance_radar/config/electronic_surveillance_radar_config.hpp"
#include "1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "1q/electronic_surveillance_radar/extension/IInterceptPipeline.h"
#include "1q/electronic_surveillance_radar/session/EsrTraceSession.h"
#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"
#include "1q/airborne_radar/session/RadarTraceSession.h"

using ArSession = airborne_radar::session::RadarSession;
using ArConfig = airborne_radar::session::RadarSessionConfig;
static_assert(!std::is_constructible<ArSession, const ArConfig&>::value,
              "RadarSession direct construction must be disabled");
static_assert(
    !std::is_constructible<ArSession, const ArConfig&,
                           airborne_radar::extension::ISignalPipeline&>::value,
    "RadarSession direct pipeline injection construction must be disabled");
static_assert(
    !std::is_constructible<ArSession, const ArConfig&,
                           airborne_radar::extension::IEnvironmentService&>::value,
    "RadarSession direct environment injection construction must be disabled");
static_assert(!std::is_constructible<ArSession, const ArConfig&,
                                     airborne_radar::extension::RadarController&>::value,
              "RadarSession direct controller injection construction must be disabled");
static_assert(
    !std::is_constructible<ArSession, const ArConfig&, airborne_radar::extension::IRadarContext&,
                           airborne_radar::extension::ISignalPipeline&,
                           airborne_radar::extension::IEnvironmentService&,
                           airborne_radar::extension::RadarController&>::value,
    "RadarSession direct full-chain injection construction must be disabled");
static_assert(std::is_same<ArSession, decltype(airborne_radar::session::RadarSessionFactory::Create(
                                         std::declval<const ArConfig&>()))>::value,
              "RadarSessionFactory::Create must return RadarSession");
static_assert(
    std::is_same<ArSession, decltype(airborne_radar::session::RadarSessionFactory::
                                         CreateWithSignalPipeline(
                                             std::declval<const ArConfig&>(),
                                             std::declval<airborne_radar::extension::
                                                              ISignalPipeline&>()))>::value,
    "RadarSessionFactory::CreateWithSignalPipeline must return RadarSession");
static_assert(
    std::is_same<ArSession, decltype(airborne_radar::session::RadarSessionFactory::
                                         CreateWithEnvironmentService(
                                             std::declval<const ArConfig&>(),
                                             std::declval<airborne_radar::extension::
                                                              IEnvironmentService&>()))>::value,
    "RadarSessionFactory::CreateWithEnvironmentService must return RadarSession");
static_assert(
    std::is_same<ArSession,
                 decltype(airborne_radar::session::RadarSessionFactory::CreateWithController(
                     std::declval<const ArConfig&>(),
                     std::declval<airborne_radar::extension::RadarController&>()))>::value,
    "RadarSessionFactory::CreateWithController must return RadarSession");
static_assert(std::is_constructible<
                  electronic_surveillance_radar::session::EsrSession,
                  electronic_surveillance_radar::session::EsrSessionConfig,
                  electronic_surveillance_radar::extension::IInterceptPipeline&>::value,
              "EsrSession must support pipeline-only injection");
static_assert(std::is_constructible<
                  electronic_surveillance_radar::session::EsrSession,
                  electronic_surveillance_radar::session::EsrSessionConfig,
                  electronic_surveillance_radar::environment::IEsrEnvironmentService&>::value,
              "EsrSession must support environment-only injection");
static_assert(std::is_constructible<
                  electronic_surveillance_radar::session::EsrSession,
                  electronic_surveillance_radar::session::EsrSessionConfig,
                  electronic_surveillance_radar::extension::EsrController&>::value,
              "EsrSession must support controller-only injection");
static_assert(std::is_constructible<
                  electronic_surveillance_radar::session::EsrSession,
                  electronic_surveillance_radar::session::EsrSessionConfig,
                  electronic_surveillance_radar::extension::IInterceptPipeline&,
                  electronic_surveillance_radar::environment::IEsrEnvironmentService&,
                  electronic_surveillance_radar::extension::EsrController&>::value,
              "EsrSession must support full-chain injection");

static_assert(
    !std::is_constructible<electro_optical_sensor::session::EosSession,
                           electro_optical_sensor::session::EosSessionConfig>::value,
    "EosSession direct construction must be disabled");
static_assert(
    !std::is_constructible<electro_optical_sensor::session::EosSession,
                           electro_optical_sensor::session::EosSessionConfig,
                           electro_optical_sensor::pipeline::IEosPipeline&>::value,
    "EosSession direct pipeline injection construction must be disabled");
static_assert(
    !std::is_constructible<electro_optical_sensor::session::EosSession,
                           electro_optical_sensor::session::EosSessionConfig,
                           electro_optical_sensor::environment::IEosEnvironmentService&>::value,
    "EosSession direct environment injection construction must be disabled");
static_assert(
    !std::is_constructible<electro_optical_sensor::session::EosSession,
                           electro_optical_sensor::session::EosSessionConfig,
                           electro_optical_sensor::extension::EosController&>::value,
    "EosSession direct controller injection construction must be disabled");
static_assert(
    std::is_same<
        electro_optical_sensor::session::EosSession,
        decltype(electro_optical_sensor::session::EosSessionFactory::Create(
            std::declval<const electro_optical_sensor::session::EosSessionConfig&>()))>::value,
    "EosSessionFactory::Create must return EosSession");
static_assert(
    std::is_same<
        electro_optical_sensor::session::EosSession,
        decltype(electro_optical_sensor::session::EosSessionFactory::CreateWithPipeline(
            std::declval<const electro_optical_sensor::session::EosSessionConfig&>(),
            std::declval<electro_optical_sensor::pipeline::IEosPipeline&>()))>::value,
    "EosSessionFactory::CreateWithPipeline must return EosSession");
static_assert(
    std::is_same<
        electro_optical_sensor::session::EosSession,
        decltype(electro_optical_sensor::session::EosSessionFactory::
                     CreateWithEnvironmentService(
                         std::declval<const electro_optical_sensor::session::EosSessionConfig&>(),
                         std::declval<electro_optical_sensor::environment::
                                          IEosEnvironmentService&>()))>::value,
    "EosSessionFactory::CreateWithEnvironmentService must return EosSession");
static_assert(
    std::is_same<
        electro_optical_sensor::session::EosSession,
        decltype(electro_optical_sensor::session::EosSessionFactory::CreateWithController(
            std::declval<const electro_optical_sensor::session::EosSessionConfig&>(),
            std::declval<electro_optical_sensor::extension::EosController&>()))>::value,
    "EosSessionFactory::CreateWithController must return EosSession");

namespace airborne_radar {
namespace {

TEST(PublicHeadersSmokeTest, StablePublicSurfaceSupportsMinimalUsage) {
  oneq::common::LlaCoordinateDegM origin_lla;
  origin_lla.latitude_deg = 0.0;
  origin_lla.longitude_deg = 0.0;
  origin_lla.altitude_m = 0.0;
  oneq::common::EcefCoordinateM origin_ecef;
  ASSERT_TRUE(oneq::common::TryLlaToEcef(origin_lla, &origin_ecef));

  session::RadarSessionConfig session_config =
      config::MakeDefaultRadarSessionConfig();
  session_config.environment_default_config.model_config.base_propagation_loss_db = 6.0f;

  session::RadarCycleInput input;
  input.dt_sec = 1.0f;
  const std::vector<session::ValidationIssue> issues =
      session::ValidateRadarCycleInput(input);

  EXPECT_FALSE(session::HasValidationError(issues));

  environment::EnvironmentSceneState scene_state;
  scene_state.jammer_emitters.push_back(environment::JammerEmitterState{});

  session::RadarSession session = session::RadarSessionFactory::Create(session_config);
  std::shared_ptr<oneq::common::trace::TraceSink> trace_sink(
      new oneq::common::trace::JsonlFileTraceSink("/tmp/oneq-smoke-radar-trace.jsonl", false));
  session::RadarTraceSession trace_session(
      session_config, session::RadarTraceSessionOptions{trace_sink, false});
  const config::RadarRuntimeConfigPatch runtime_patch =
      config::RadarRuntimeConfigBuilder()
          .WithRadarWorkSubMode(model::RadarWorkSubMode::kTas)
          .EnableCommandedBeamwidth(true)
          .Build();
  session.ApplyRuntimeConfig(runtime_patch);
  const session::RadarCycleResult result = session.StepWithResult(input, scene_state);
  const session::RadarCycleResult trace_result = trace_session.StepWithResult(input, scene_state);
  const std::size_t confirmed_tracks = output::CountTracksByStatus(
      result.track_output_frame, model::DecisionTrackStatus::kConfirmed);

  EXPECT_GE(confirmed_tracks, 0U);
  EXPECT_GE(result.association_quality_metrics.detection_count, 0U);
  EXPECT_GE(trace_result.association_quality_metrics.detection_count, 0U);
}

TEST(PublicHeadersSmokeTest, RadarSessionBuilderCanConfigureLeafAndDomainFields) {
  model::AzimuthElevationDeg scan_center;
  scan_center.az_deg = -12.0f;
  scan_center.el_deg = 6.0f;

  const session::RadarSessionConfig config =
      config::RadarSessionConfigBuilder()
          .Detection()
          .EnablePhysicsDetection(true)
          .WithPulseCount(16)
          .WithPeakPowerW(5e6f)
          .WithFrequencyHz(9.3e9f)
          .WithMainBeamGainDb(38.0f)
          .WithNoiseFigureDb(3.5f)
          .End()
          .Beam()
          .WithScanCenterDeg(scan_center)
          .End()
          .Lifecycle()
          .WithLifecycleConfirmHits(2U)
          .End()
          .Environment()
          .WithJammingDetectionThresholdDb(8.0f)
          .End()
          .Build();
  EXPECT_TRUE(config.detection.enable_physics_detection);
  EXPECT_EQ(config.detection.pulse_count, 16);
  EXPECT_FLOAT_EQ(config.detection.transmitter.peak_power_w, 5e6f);
  EXPECT_FLOAT_EQ(config.detection.transmitter.frequency_hz, 9.3e9f);
  EXPECT_FLOAT_EQ(config.detection.antenna.main_beam_gain_db, 38.0f);
  EXPECT_FLOAT_EQ(config.detection.receiver.noise_figure_db, 3.5f);
  EXPECT_FLOAT_EQ(config.beam_control.radar_orientation.scan_center_deg.az_deg, -12.0f);
  EXPECT_EQ(config.lifecycle.lifecycle_config.confirm_hits, 2U);
  EXPECT_FLOAT_EQ(config.environment_default_config.jamming_detection_threshold_db, 8.0f);

  environment::EnvironmentModelConfig model;
  model.base_propagation_loss_db = 5.0f;
  environment::EnvironmentDefaultConfig env =
      environment::EnvironmentDefaultConfigBuilder()
          .WithModelConfig(model)
          .Build();
  env.jamming_detection_threshold_db = 8.0f;
  EXPECT_FLOAT_EQ(env.model_config.base_propagation_loss_db, 5.0f);
  EXPECT_FLOAT_EQ(env.jamming_detection_threshold_db, 8.0f);
}

}  // namespace
}  // namespace airborne_radar

namespace electronic_surveillance_radar {
namespace {

TEST(PublicHeadersSmokeTest, EsrPublicSurfaceSupportsMinimalUsage) {
  const oneq::common::ScanStartPosition shared_start = oneq::common::ScanStartPosition::kLeftTop;
  EXPECT_EQ(static_cast<int>(shared_start), 0);

  session::EsrSessionConfig session_config = config::EsrSessionConfigBuilder()
                                                       .EnableLayeredConfig(true)
                                                       .WithScanRateHz(1.0f)
                                                       .EnableSpectralAnalysis(true)
                                                       .Build();
  session_config.pipeline_config.scan.scan_start_az_deg = -60.0f;
  session_config.pipeline_config.scan.scan_end_az_deg = 60.0f;
  session_config.pipeline_config.scan.scan_start_el_deg = -20.0f;
  session_config.pipeline_config.scan.scan_end_el_deg = 20.0f;
  session_config.pipeline_config.scan.az_step_deg = 120.0f;
  session_config.pipeline_config.scan.el_step_deg = 40.0f;

  session::EsrCycleInput input;
  input.cycle_index = 4U;
  input.dt_sec = 1.0f;
  model::EmitterTruthState emitter;
  emitter.emitter_id = "smoke-emitter";
  emitter.pose.position_m.x = 1200.0f;
  emitter.carrier_hz = 10.0e9;
  emitter.bandwidth_hz = 2.0e6;
  emitter.tx_power_w = 5.0e7;
  emitter.pulse_width_s = 1.0e-6;
  emitter.pri_s = 1.0e-4;
  input.scene_emitters.push_back(emitter);

  model::EsrCoordinateReference esr_reference;
  esr_reference.origin_lla.latitude_deg = 0.0;
  esr_reference.origin_lla.longitude_deg = 0.0;
  esr_reference.origin_lla.altitude_m = 0.0;
  model::EsrVector3f esr_local_position;
  ASSERT_TRUE(model::TryConvertLlaToEsrLocal(esr_reference.origin_lla, esr_reference,
                                               &esr_local_position));

  const session::EsrValidationIssueList issues = session::ValidateEsrCycleInput(input);
  EXPECT_FALSE(session::HasEsrValidationError(issues));

  session::EsrSession session(session_config);
  const session::EsrRuntimeConfigPatch runtime_patch =
      config::EsrRuntimeConfigBuilder().WithDetectionMinSnrDb(5.0f).Build();
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
  session::EosSessionConfig session_config = config::EosSessionConfigBuilder()
                                                       .WithWorkMode(session::EosWorkMode::kFused)
                                                       .WithMinimumSnrDb(0.0f)
                                                       .Build();
  session_config.scan_start_az_deg = -20.0f;
  session_config.scan_end_az_deg = 20.0f;

  session::EosCycleInput input;
  input.cycle_index = 2U;
  input.dt_sec = 1.0f;
  input.day_night_type = session::DayNightType::kDay;
  session::EosTargetState target;
  target.target_id = 7U;
  target.range_m = 1500.0f;
  target.azimuth_deg = 0.0f;
  target.elevation_deg = 0.0f;
  target.apparent_temperature_k = 320.0f;
  target.emissivity = 0.9f;
  target.reflectance = 0.4f;
  target.projected_area_m2 = 2.0f;
  input.scene_targets.push_back(target);

  utils::EosCoordinateReference eos_reference;
  eos_reference.origin_lla.latitude_deg = 0.0;
  eos_reference.origin_lla.longitude_deg = 0.0;
  eos_reference.origin_lla.altitude_m = 0.0;
  oneq::common::Vector3f eos_local_position;
  ASSERT_TRUE(utils::TryConvertLlaToEosLocal(eos_reference.origin_lla, eos_reference,
                                                      &eos_local_position));

  const model::EosValidationIssueList issues = model::ValidateEosCycleInput(input);
  EXPECT_FALSE(model::HasEosValidationError(issues));

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
  const session::EosRuntimeConfigPatch runtime_patch =
      config::EosRuntimeConfigBuilder().WithFrameRateHz(15.0f).Build();
  session.ApplyRuntimeConfig(runtime_patch);
  const model::EosCycleResult result = session.StepWithResult(input);
  session::EosTraceSession trace_session(session_config, session::EosTraceSessionOptions{});
  const model::EosCycleResult trace_result = trace_session.StepWithResult(input);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_FALSE(result.reused_previous_output);
  EXPECT_TRUE(trace_result.executed_this_cycle);
  EXPECT_FALSE(trace_result.reused_previous_output);
  EXPECT_GE(result.output_frame.detections.size(), 0U);
  EXPECT_GE(trace_result.output_frame.detections.size(), 0U);
}

}  // namespace
}  // namespace electro_optical_sensor
