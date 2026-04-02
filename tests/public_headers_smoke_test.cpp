/**
 * @file public_headers_smoke_test.cpp
 * @brief 验证稳定公共头集合可被统一包含并完成最小用法编译。
 */

#include <gtest/gtest.h>

#include <memory>

#include "1q/airborne_radar/common/control/ControlDirective.h"
#include "1q/airborne_radar/common/control/RadarCommand.h"
#include "1q/airborne_radar/common/control/RadarControlProfile.h"
#include "1q/airborne_radar/common/model/DecisionInputFrame.h"
#include "1q/airborne_radar/common/model/DecisionSourceInfo.h"
#include "1q/airborne_radar/common/model/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/common/model/TargetCategory.h"
#include "1q/airborne_radar/common/model/TargetFeature.h"
#include "1q/airborne_radar/common/output/TrackOutputFrame.h"
#include "1q/airborne_radar/common/output/TrackOutputQueries.h"
#include "1q/airborne_radar/common/utils/JammingSemantics.h"
#include "1q/airborne_radar/common/utils/RadarOrientationUtils.h"
#include "1q/airborne_radar/common/utils/TargetFeatureUtils.h"
#include "1q/airborne_radar/config/AntennaPatternConfig.h"
#include "1q/airborne_radar/config/ConfigPresets.h"
#include "1q/airborne_radar/config/RadarOrientationConfig.h"
#include "1q/airborne_radar/config/RadarRuntimeConfigBuilder.h"
#include "1q/airborne_radar/config/RadarSessionConfigBuilder.h"
#include "1q/airborne_radar/config/SignalBeamControlConfig.h"
#include "1q/airborne_radar/config/SignalDetectionConfig.h"
#include "1q/airborne_radar/config/SignalPipelineConfig.h"
#include "1q/airborne_radar/core/context/IRadarContext.h"
#include "1q/airborne_radar/core/context/RadarCycleInput.h"
#include "1q/airborne_radar/core/context/RadarInputValidation.h"
#include "1q/airborne_radar/core/controller/IRadarOutputReader.h"
#include "1q/airborne_radar/core/controller/RadarController.h"
#include "1q/airborne_radar/core/session/RadarCycleResult.h"
#include "1q/airborne_radar/core/session/RadarSession.h"
#include "1q/airborne_radar/decision/pipeline/ControlReducerTypes.h"
#include "1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"
#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "1q/airborne_radar/signal/pipeline/SignalPipelineResultTypes.h"
#include "1q/api.hpp"
#include "1q/common/coordinate_transform.h"
#include "1q/common/pose_types.h"
#include "1q/common/scan_schedule_types.h"
#include "1q/common/trace/TraceSink.h"
#include "1q/electro_optical_sensor/common/EosOutputFrame.h"
#include "1q/electro_optical_sensor/foundation/EosOpticalCharacteristics.h"
#include "1q/electro_optical_sensor/foundation/EosPropagation.h"
#include "1q/electro_optical_sensor/foundation/EosRadiometry.h"
#include "1q/electro_optical_sensor/core/context/EosCycleInput.h"
#include "1q/electro_optical_sensor/core/context/EosCoordinateUtils.h"
#include "1q/electro_optical_sensor/core/context/EosInputValidation.h"
#include "1q/electro_optical_sensor/core/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/core/session/EosSession.h"
#include "1q/electro_optical_sensor/tools/EosTraceSession.h"
#include "1q/electronic_surveillance_radar/common/EmitterTruthState.h"
#include "1q/electronic_surveillance_radar/common/EsrCoordinateUtils.h"
#include "1q/electronic_surveillance_radar/core/context/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/core/context/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/core/session/EsrSession.h"
#include "1q/electronic_surveillance_radar/tools/EsrTraceSession.h"
#include "1q/airborne_radar/tools/RadarTraceSession.h"

namespace airborne_radar {
namespace {

TEST(PublicHeadersSmokeTest, StablePublicSurfaceSupportsMinimalUsage) {
  oneq::common::LlaCoordinateDegM origin_lla;
  origin_lla.latitude_deg = 0.0;
  origin_lla.longitude_deg = 0.0;
  origin_lla.altitude_m = 0.0;
  oneq::common::EcefCoordinateM origin_ecef;
  ASSERT_TRUE(oneq::common::TryLlaToEcef(origin_lla, &origin_ecef));

  core::session::RadarSessionConfig session_config =
      common::config::MakeDefaultRadarSessionConfig();
  session_config.environment_model_config.base_propagation_loss_db = 6.0f;
  session_config.signal_pipeline_config.lifecycle.enable_auto_lifecycle_manager = true;

  core::context::RadarCycleInput input;
  input.dt_sec = 1.0f;
  const std::vector<core::context::ValidationIssue> issues =
      core::context::ValidateRadarCycleInput(input);

  EXPECT_FALSE(core::context::HasValidationError(issues));

  environment::EnvironmentSceneState scene_state;
  scene_state.jammer_emitters.push_back(environment::JammerEmitterState{});

  core::session::RadarSession session(session_config);
  std::shared_ptr<oneq::common::trace::TraceSink> trace_sink(
      new oneq::common::trace::JsonlFileTraceSink("/tmp/oneq-smoke-radar-trace.jsonl", false));
  tools::RadarTraceSession trace_session(
      session_config, tools::RadarTraceSessionOptions{trace_sink, false});
  const common::config::RadarRuntimeConfigPatch runtime_patch =
      common::config::RadarRuntimeConfigBuilder()
          .WithRadarWorkSubMode(common::config::RadarWorkSubMode::kTas)
          .EnableCommandedBeamwidth(true)
          .Build();
  session.ApplyRuntimeConfig(runtime_patch);
  const core::session::RadarCycleResult result = session.StepWithResult(input, scene_state);
  const core::session::RadarCycleResult trace_result = trace_session.StepWithResult(input, scene_state);
  const std::size_t confirmed_tracks = common::output::CountTracksByStatus(
      result.track_output_frame, common::model::DecisionTrackStatus::kConfirmed);

  EXPECT_GE(confirmed_tracks, 0U);
  EXPECT_GE(result.association_quality_metrics.detection_count, 0U);
  EXPECT_GE(trace_result.association_quality_metrics.detection_count, 0U);
}

}  // namespace
}  // namespace airborne_radar

namespace electronic_surveillance_radar {
namespace {

TEST(PublicHeadersSmokeTest, EsrPublicSurfaceSupportsMinimalUsage) {
  const oneq::common::ScanStartPosition shared_start = oneq::common::ScanStartPosition::kLeftTop;
  EXPECT_EQ(static_cast<int>(shared_start), 0);

  core::session::EsrSessionConfig session_config;
  session_config.pipeline_config.scan.scan_start_az_deg = -60.0f;
  session_config.pipeline_config.scan.scan_end_az_deg = 60.0f;
  session_config.pipeline_config.scan.scan_start_el_deg = -20.0f;
  session_config.pipeline_config.scan.scan_end_el_deg = 20.0f;
  session_config.pipeline_config.scan.az_step_deg = 120.0f;
  session_config.pipeline_config.scan.el_step_deg = 40.0f;

  core::context::EsrCycleInput input;
  input.cycle_index = 4U;
  input.dt_sec = 1.0f;
  common::EmitterTruthState emitter;
  emitter.emitter_id = "smoke-emitter";
  emitter.pose.position_m.x = 1200.0f;
  emitter.carrier_hz = 10.0e9;
  emitter.bandwidth_hz = 2.0e6;
  emitter.tx_power_w = 5.0e7;
  emitter.pulse_width_s = 1.0e-6;
  emitter.pri_s = 1.0e-4;
  input.scene_emitters.push_back(emitter);

  common::EsrCoordinateReference esr_reference;
  esr_reference.origin_lla.latitude_deg = 0.0;
  esr_reference.origin_lla.longitude_deg = 0.0;
  esr_reference.origin_lla.altitude_m = 0.0;
  common::EsrVector3f esr_local_position;
  ASSERT_TRUE(common::TryConvertLlaToEsrLocal(esr_reference.origin_lla, esr_reference,
                                               &esr_local_position));

  const core::context::EsrValidationIssueList issues = core::context::ValidateEsrCycleInput(input);
  EXPECT_FALSE(core::context::HasEsrValidationError(issues));

  core::session::EsrSession session(session_config);
  const core::session::EsrCycleResult result = session.StepWithResult(input);
  tools::EsrTraceSession trace_session(session_config, tools::EsrTraceSessionOptions{});
  const core::session::EsrCycleResult trace_result = trace_session.StepWithResult(input);

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
  core::session::EosSessionConfig session_config;
  session_config.work_mode = core::session::EosWorkMode::kFused;
  session_config.minimum_snr_db = 0.0f;
  session_config.scan_start_az_deg = -20.0f;
  session_config.scan_end_az_deg = 20.0f;

  core::context::EosCycleInput input;
  input.cycle_index = 2U;
  input.dt_sec = 1.0f;
  input.day_night_type = core::context::DayNightType::kDay;
  core::context::EosTargetState target;
  target.target_id = 7U;
  target.range_m = 1500.0f;
  target.azimuth_deg = 0.0f;
  target.elevation_deg = 0.0f;
  target.apparent_temperature_k = 320.0f;
  target.emissivity = 0.9f;
  target.reflectance = 0.4f;
  target.projected_area_m2 = 2.0f;
  input.scene_targets.push_back(target);

  core::context::EosCoordinateReference eos_reference;
  eos_reference.origin_lla.latitude_deg = 0.0;
  eos_reference.origin_lla.longitude_deg = 0.0;
  eos_reference.origin_lla.altitude_m = 0.0;
  oneq::common::Vector3f eos_local_position;
  ASSERT_TRUE(core::context::TryConvertLlaToEosLocal(eos_reference.origin_lla, eos_reference,
                                                      &eos_local_position));

  const core::context::EosValidationIssueList issues = core::context::ValidateEosCycleInput(input);
  EXPECT_FALSE(core::context::HasEosValidationError(issues));

  const float diffraction_rad =
      foundation::optics::ComputeDiffractionLimitedAngularResolutionRad(4.0f, 0.2f);
  const float transmittance =
      foundation::propagation::ComputeAtmosphericTransmittance(2.0e-5f, 1.0e-5f, 1500.0f);
  const float planck_radiance = foundation::radiometry::ComputePlanckRadiance(4.0f, 320.0f);
  EXPECT_GT(diffraction_rad, 0.0f);
  EXPECT_GT(transmittance, 0.0f);
  EXPECT_GT(planck_radiance, 0.0f);

  core::session::EosSession session(session_config);
  const core::session::EosCycleResult result = session.StepWithResult(input);
  tools::EosTraceSession trace_session(session_config, tools::EosTraceSessionOptions{});
  const core::session::EosCycleResult trace_result = trace_session.StepWithResult(input);
  EXPECT_GE(result.output_frame.detections.size(), 0U);
  EXPECT_GE(trace_result.output_frame.detections.size(), 0U);
}

}  // namespace
}  // namespace electro_optical_sensor
