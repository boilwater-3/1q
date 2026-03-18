// Copyright 2026. All Rights Reserved.
//
// @file RadarSession.cpp
// @brief 实现面向外部接入的雷达高层会话门面。

#include "1q/airborne_radar/core/session/RadarSession.h"

#include "1q/airborne_radar/core/context/MutableRadarContext.h"
#include "1q/airborne_radar/core/controller/RadarController.h"

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
  impl_->radar_context.BeginCycle(input);
  impl_->controller.RunOnce();
  if (!impl_->controller.HasLatestTrackOutputFrame()) {
    return common::TrackOutputFrame();
  }
  return impl_->controller.GetLatestTrackOutputFrame();
}

common::TrackOutputFrame RadarSession::Step(
    const context::RadarCycleInput& input,
    const environment::EnvironmentSceneState& scene_state) {
  impl_->environment_service.UpdateSceneState(scene_state);
  return Step(input);
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

std::vector<signal::tracking::TrackMeasurement>
RadarSession::GetLastTrackMeasurements() const {
  return impl_->signal_pipeline.GetLastTrackMeasurements();
}

void RadarSession::UpdateSignalPipelineConfig(
    const signal::pipeline::SignalPipelineConfig& config) {
  impl_->signal_pipeline.UpdateConfig(config);
  impl_->controller.SetTrackLifecycleManager(nullptr);
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
