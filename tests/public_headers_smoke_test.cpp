/**
 * @file public_headers_smoke_test.cpp
 * @brief 验证稳定公共头集合可被统一包含并完成最小用法编译。
 */

#include <gtest/gtest.h>

#include "1q/common/pose_types.h"
#include "1q/common/scan_schedule_types.h"
#include "1q/airborne_radar/common/config/AntennaPatternConfig.h"
#include "1q/airborne_radar/common/utils/AntennaPatternUtils.h"
#include "1q/airborne_radar/common/config/ConfigPresets.h"
#include "1q/airborne_radar/common/control/ControlDirective.h"
#include "1q/airborne_radar/common/model/DecisionInputFrame.h"
#include "1q/airborne_radar/common/model/DecisionSourceInfo.h"
#include "1q/airborne_radar/common/model/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/common/utils/JammingSemantics.h"
#include "1q/airborne_radar/common/control/RadarCommand.h"
#include "1q/airborne_radar/common/control/RadarControlProfile.h"
#include "1q/airborne_radar/common/config/RadarOrientationConfig.h"
#include "1q/airborne_radar/common/utils/RadarOrientationUtils.h"
#include "1q/airborne_radar/common/model/TargetCategory.h"
#include "1q/airborne_radar/common/model/TargetFeature.h"
#include "1q/airborne_radar/common/utils/TargetFeatureUtils.h"
#include "1q/airborne_radar/common/output/TrackOutputFrame.h"
#include "1q/airborne_radar/common/output/TrackOutputQueries.h"
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
#include "1q/airborne_radar/signal/detection/DetectionTypes.h"
#include "1q/airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "1q/airborne_radar/signal/pipeline/SignalPipelineTypes.h"
#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/common/EmitterTruthState.h"
#include "1q/electronic_surveillance_radar/core/context/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/core/context/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/core/session/EsrSession.h"

namespace airborne_radar {
namespace {

TEST(PublicHeadersSmokeTest, StablePublicSurfaceSupportsMinimalUsage) {
  core::session::RadarSessionConfig session_config = common::config::MakeDefaultRadarSessionConfig();
  session_config.environment_model_config.base_propagation_loss_db = 6.0f;
  session_config.signal_pipeline_config.lifecycle.enable_auto_lifecycle_manager = true;
  session_config.signal_pipeline_config.lifecycle.lifecycle_config.imm_activation_policy =
      signal::pipeline::ImmActivationPolicy::kConfirmedTracksOnly;

  core::context::RadarCycleInput input;
  input.dt_sec = 1.0f;
  const std::vector<core::context::ValidationIssue> issues =
      core::context::ValidateRadarCycleInput(input);

  EXPECT_FALSE(core::context::HasValidationError(issues));

  environment::EnvironmentSceneState scene_state;
  scene_state.jammer_emitters.push_back(environment::JammerEmitterState{});

  core::session::RadarSession session(session_config);
  const core::session::RadarCycleResult result = session.StepWithResult(input, scene_state);
  const std::size_t confirmed_tracks = common::output::CountTracksByStatus(
      result.track_output_frame, common::model::DecisionTrackStatus::kConfirmed);

  EXPECT_GE(confirmed_tracks, 0U);
  EXPECT_GE(result.association_quality_metrics.detection_count, 0U);
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

  const core::context::EsrValidationIssueList issues = core::context::ValidateEsrCycleInput(input);
  EXPECT_FALSE(core::context::HasEsrValidationError(issues));

  core::session::EsrSession session(session_config);
  const core::session::EsrCycleResult result = session.StepWithResult(input);

  EXPECT_GE(result.output_frame.observation_output.observations.size(), 0U);
  EXPECT_GE(result.output_frame.emitter_output.hypotheses.size(), 0U);
  EXPECT_GE(result.output_frame.truth_evaluation_output.associations.size(), 0U);
}

}  // namespace
}  // namespace electronic_surveillance_radar
