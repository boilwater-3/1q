#include "1q/airborne_radar/core/session/RadarSession.h"

#include "airborne_radar/core/context/MutableRadarContext.h"
#include "1q/airborne_radar/core/controller/RadarController.h"
#include "airborne_radar/environment/EnvironmentService.h"
#include "airborne_radar/signal/pipeline/SignalPipeline.h"

namespace airborne_radar {
namespace core {
namespace session {

struct RadarSession::Impl {
  explicit Impl(const RadarSessionConfig& config)
      : signal_pipeline(config.signal_pipeline_config),
        environment_service(config.environment_model_config),
        controller(radar_context, signal_pipeline, environment_service) {
    environment_service.SetJammingDetectionThresholdDb(
        config.jamming_detection_threshold_db);
  }
  /**
   * @brief 收集当前周期的聚合结果。 
   * @return 当前 Session 运行态导出的聚合结果。 
   */
  RadarCycleResult BuildCycleResult() const {
    RadarCycleResult result;
    if (controller.HasLatestTrackOutputFrame()) {
      result.track_output_frame = controller.GetLatestTrackOutputFrame();
    }
    result.submitted_commands = radar_context.GetSubmittedCommands();
    result.has_control_profile = radar_context.HasLatestControlProfile();
    if (result.has_control_profile) {
      result.control_profile = radar_context.GetLatestControlProfile();
    }
    result.association_quality_metrics =
        signal_pipeline.GetLastAssociationQualityMetrics();
    return result;
  }

  context::MutableRadarContext radar_context{};
  signal::pipeline::SignalPipeline signal_pipeline;
  environment::EnvironmentService environment_service;
  controller::RadarController controller;
};

RadarSession::RadarSession(RadarSessionConfig config)
    : impl_(new Impl(config)) {}

RadarSession::~RadarSession() = default;

common::TrackOutputFrame RadarSession::Step(
    const context::RadarCycleInput& input) {
  return StepWithResult(input).track_output_frame;
}

common::TrackOutputFrame RadarSession::Step(
    const context::RadarCycleInput& input,
    const environment::EnvironmentSceneState& scene_state) {
  return StepWithResult(input, scene_state).track_output_frame;
}

RadarCycleResult RadarSession::StepWithResult(
    const context::RadarCycleInput& input) {
  impl_->radar_context.BeginCycle(input);
  impl_->controller.RunOnce();
  return impl_->BuildCycleResult();
}

RadarCycleResult RadarSession::StepWithResult(
    const context::RadarCycleInput& input,
    const environment::EnvironmentSceneState& scene_state) {
  impl_->environment_service.UpdateSceneState(scene_state);
  return StepWithResult(input);
}

const std::vector<common::RadarCommand>&
RadarSession::GetSubmittedCommands() const {
  return impl_->radar_context.GetSubmittedCommands();
}

bool RadarSession::HasLatestControlProfile() const {
  return impl_->radar_context.HasLatestControlProfile();
}

const common::RadarControlProfile& RadarSession::GetLatestControlProfile() const {
  return impl_->radar_context.GetLatestControlProfile();
}

signal::pipeline::AssociationQualityMetrics
RadarSession::GetLastAssociationQualityMetrics() const {
  return impl_->signal_pipeline.GetLastAssociationQualityMetrics();
}

void RadarSession::UpdateSignalPipelineConfig(
    const signal::pipeline::SignalPipelineConfig& config) {
  impl_->signal_pipeline.UpdateConfig(config);
}

void RadarSession::UpdateEnvironmentModelConfig(
    const environment::EnvironmentModelConfig& config) {
  impl_->environment_service.UpdateModelConfig(config);
}

void RadarSession::SetJammingDetectionThresholdDb(float threshold_db) {
  impl_->environment_service.SetJammingDetectionThresholdDb(threshold_db);
}

} // namespace session
} // namespace core
} // namespace airborne_radar


