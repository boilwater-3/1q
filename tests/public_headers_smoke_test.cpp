/**
 * @file public_headers_smoke_test.cpp
 * @brief 验证稳定公共头集合可被统一包含并完成最小用法编译。
 */

#include <gtest/gtest.h>

#include "1q/api.hpp"
#include "1q/airborne_radar/common/AntennaPatternConfig.h"
#include "1q/airborne_radar/common/AntennaPatternUtils.h"
#include "1q/airborne_radar/common/ConfigPresets.h"
#include "1q/airborne_radar/common/ControlDirective.h"
#include "1q/airborne_radar/common/DecisionInputFrame.h"
#include "1q/airborne_radar/common/DecisionSourceInfo.h"
#include "1q/airborne_radar/common/DecisionTrackSnapshot.h"
#include "1q/airborne_radar/common/JammingSemantics.h"
#include "1q/airborne_radar/common/RadarCommand.h"
#include "1q/airborne_radar/common/RadarControlProfile.h"
#include "1q/airborne_radar/common/RadarOrientationConfig.h"
#include "1q/airborne_radar/common/RadarOrientationUtils.h"
#include "1q/airborne_radar/common/TargetCategory.h"
#include "1q/airborne_radar/common/TargetFeature.h"
#include "1q/airborne_radar/common/TargetFeatureUtils.h"
#include "1q/airborne_radar/common/TrackOutputFrame.h"
#include "1q/airborne_radar/core/context/IRadarContext.h"
#include "1q/airborne_radar/core/context/RadarCycleInput.h"
#include "1q/airborne_radar/core/context/RadarInputValidation.h"
#include "1q/airborne_radar/core/controller/RadarController.h"
#include "1q/airborne_radar/core/output/IRadarOutputReader.h"
#include "1q/airborne_radar/core/output/TrackOutputQueries.h"
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

namespace airborne_radar {
namespace {

TEST(PublicHeadersSmokeTest, StablePublicSurfaceSupportsMinimalUsage) {
  core::session::RadarSessionConfig session_config =
      common::MakeDefaultRadarSessionConfig();
  session_config.environment_model_config.base_propagation_loss_db = 6.0f;
  session_config.signal_pipeline_config.lifecycle.enable_auto_lifecycle_manager =
      true;
  session_config.signal_pipeline_config.lifecycle.lifecycle_config
      .imm_activation_policy =
      signal::pipeline::ImmActivationPolicy::kConfirmedTracksOnly;

  core::context::RadarCycleInput input;
  input.dt_sec = 1.0f;
  const std::vector<core::context::ValidationIssue> issues =
      core::context::ValidateRadarCycleInput(input);

  EXPECT_FALSE(core::context::HasValidationError(issues));

  environment::EnvironmentSceneState scene_state;
  scene_state.jammer_emitters.push_back(environment::JammerEmitterState{});

  core::session::RadarSession session(session_config);
  const core::session::RadarCycleResult result =
      session.StepWithResult(input, scene_state);
  const std::size_t confirmed_tracks =
      core::output::CountTracksByStatus(
          result.track_output_frame,
          common::DecisionTrackStatus::kConfirmed);

  EXPECT_GE(confirmed_tracks, 0U);
  EXPECT_GE(result.association_quality_metrics.detection_count, 0U);
}

} // namespace
} // namespace airborne_radar
