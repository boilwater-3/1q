/**
 * @file ar_extension_consumer.cpp
 * @brief 验证安装后机载雷达扩展接口可被外部工程实现并接入控制器。
 */

#include <vector>

#include "1q/airborne_radar/common/output/TrackOutputFrame.h"
#include "1q/airborne_radar/core/context/IRadarContext.h"
#include "1q/airborne_radar/core/controller/RadarController.h"
#include "1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/signal/pipeline/ISignalPipeline.h"

namespace airborne_radar {
namespace {

class DummyRadarContext : public core::context::IRadarContext {
 public:
  void BeginCycle(const core::context::RadarCycleInput& input) override {
    (void)input;
    commands_.clear();
  }

  const common::model::TargetFeatureList& GetTargetFeatures() const override {
    static const common::model::TargetFeatureList kEmptyTargets;
    return kEmptyTargets;
  }

  common::model::PlatformAttitudeDeg GetPlatformAttitude() const override {
    return platform_attitude_;
  }

  float GetCycleDeltaTimeSec() const override { return 1.0f; }

  void SubmitControlCommand(common::control::RadarCommand cmd) override {
    commands_.push_back(cmd);
  }

  void UpdateRadarControlProfile(const common::control::RadarControlProfile& profile) override {
    control_profile_ = profile;
    has_control_profile_ = true;
  }

  const std::vector<common::control::RadarCommand>& GetSubmittedCommands() const override {
    return commands_;
  }

  bool HasLatestControlProfile() const override { return has_control_profile_; }

  const common::control::RadarControlProfile& GetLatestControlProfile() const override {
    return control_profile_;
  }

 private:
  common::model::PlatformAttitudeDeg platform_attitude_{};
  common::control::RadarControlProfile control_profile_{};
  std::vector<common::control::RadarCommand> commands_{};
  bool has_control_profile_{false};
};

class DummyEnvironmentService : public environment::IEnvironmentService {
 public:
  void BeginCycle(const environment::EnvironmentCycleContext& cycle_context) override {
    cycle_context_ = cycle_context;
  }

  environment::EnvironmentSnapshot SampleEnvironment() const override {
    environment::EnvironmentSnapshot snapshot;
    snapshot.cycle_dt_sec = cycle_context_.dt_sec;
    return snapshot;
  }

  void UpdateSceneState(const environment::EnvironmentSceneState& scene_state) override {
    (void)scene_state;
  }

  void UpdateModelConfig(const environment::EnvironmentModelConfig& config) override {
    (void)config;
  }

  void SetJammingDetectionThresholdDb(float threshold_db) override { (void)threshold_db; }

 private:
  environment::EnvironmentCycleContext cycle_context_{};
};

class DummySignalPipeline : public signal::pipeline::ISignalPipeline {
 public:
  signal::pipeline::SignalCycleResult RunCycle(
      const common::model::TargetFeatureList& input_state,
      const environment::IEnvironmentService& environment) override {
    (void)environment;
    signal::pipeline::SignalCycleResult result;
    result.updated_features = input_state;
    return result;
  }

  void UpdatePlatformAttitude(
      const common::model::PlatformAttitudeDeg& platform_attitude_deg) override {
    platform_attitude_ = platform_attitude_deg;
  }

  common::model::PlatformAttitudeDeg GetPlatformAttitude() const override {
    return platform_attitude_;
  }

  void SetControlProfile(const common::control::RadarControlProfile& control_profile) override {
    control_profile_ = control_profile;
  }

  common::control::RadarControlProfile GetControlProfile() const override {
    return control_profile_;
  }

  void UpdateConfig(signal::config::SignalPipelineConfig config) override { config_ = config; }

  signal::pipeline::AssociationQualityMetrics GetLastAssociationQualityMetrics() const override {
    return {};
  }

 private:
  common::model::PlatformAttitudeDeg platform_attitude_{};
  common::control::RadarControlProfile control_profile_{};
  signal::config::SignalPipelineConfig config_{};
};

class DummyDecisionEngine : public decision::pipeline::ITacticalDecisionEngine {
 public:
  decision::pipeline::TacticalDecisionResult Evaluate(
      const common::model::DecisionInputFrame& input_frame,
      decision::pipeline::TacticalStateStore& state_store) override {
    (void)input_frame;
    (void)state_store;
    return {};
  }
};

}  // namespace
}  // namespace airborne_radar

int main() {
  airborne_radar::DummyRadarContext radar_context;
  airborne_radar::DummyEnvironmentService environment_service;
  airborne_radar::DummySignalPipeline signal_pipeline;
  airborne_radar::DummyDecisionEngine decision_engine;

  airborne_radar::core::controller::RadarController controller(
      radar_context, signal_pipeline, decision_engine, environment_service);
  controller.UpdateControlReducerConfig({});
  controller.RunOnce();

  return controller.HasLatestTrackOutputFrame() ? 0 : 1;
}
