/**
 * @file ar_extension_consumer.cpp
 * @brief 验证安装后机载雷达扩展接口可被外部工程实现并接入控制器。
 */

#include <vector>

#include "1q/airborne_radar/output/TrackOutputFrame.h"
#include "1q/airborne_radar/extension/IRadarContext.h"
#include "1q/airborne_radar/extension/RadarController.h"
#include "1q/airborne_radar/extension/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/extension/ISignalPipeline.h"

namespace airborne_radar {
namespace {

class DummyRadarContext : public extension::IRadarContext {
 public:
  void BeginCycle(const session::RadarCycleInput& input) override {
    (void)input;
    commands_.clear();
  }

  const model::TargetFeatureList& GetTargetFeatures() const override {
    static const model::TargetFeatureList kEmptyTargets;
    return kEmptyTargets;
  }

  model::PlatformAttitudeDeg GetPlatformAttitude() const override {
    return platform_attitude_;
  }

  float GetCycleDeltaTimeSec() const override { return 1.0f; }

  void SubmitControlCommand(extension::control::RadarCommand cmd) override {
    commands_.push_back(cmd);
  }

  void UpdateRadarControlProfile(const extension::control::RadarControlProfile& profile) override {
    control_profile_ = profile;
    has_control_profile_ = true;
  }

  const std::vector<extension::control::RadarCommand>& GetSubmittedCommands() const override {
    return commands_;
  }

  bool HasLatestControlProfile() const override { return has_control_profile_; }

  const extension::control::RadarControlProfile& GetLatestControlProfile() const override {
    return control_profile_;
  }

  extension::RadarContextRuntimeState CaptureRuntimeState() const override {
    extension::RadarContextRuntimeState state;
    state.platform_attitude_deg = platform_attitude_;
    state.submitted_commands = commands_;
    state.latest_control_profile = control_profile_;
    state.has_latest_control_profile = has_control_profile_;
    return state;
  }

  void RestoreRuntimeState(const extension::RadarContextRuntimeState& state) override {
    platform_attitude_ = state.platform_attitude_deg;
    commands_ = state.submitted_commands;
    control_profile_ = state.latest_control_profile;
    has_control_profile_ = state.has_latest_control_profile;
  }

 private:
  model::PlatformAttitudeDeg platform_attitude_{};
  extension::control::RadarControlProfile control_profile_{};
  std::vector<extension::control::RadarCommand> commands_{};
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

  environment::EnvironmentSceneState GetPendingSceneState() const override {
    return environment::EnvironmentSceneState();
  }

  void UpdateModelConfig(const environment::EnvironmentModelConfig& config) override {
    (void)config;
  }

  void SetJammingSensitivityProfile(environment::JammingSensitivityProfile profile) override {
    (void)profile;
  }

  environment::EnvironmentServiceRuntimeState CaptureRuntimeState() const override {
    environment::EnvironmentServiceRuntimeState state;
    state.active_cycle_context = cycle_context_;
    return state;
  }

  void RestoreRuntimeState(const environment::EnvironmentServiceRuntimeState& state) override {
    cycle_context_ = state.active_cycle_context;
  }

 private:
  environment::EnvironmentCycleContext cycle_context_{};
};

class DummySignalPipeline : public extension::ISignalPipeline {
 public:
  extension::SignalCycleResult RunCycle(
      const model::TargetFeatureList& input_state,
      const environment::IEnvironmentService& environment) override {
    (void)environment;
    extension::SignalCycleResult result;
    result.executed_this_cycle = true;
    result.updated_features = input_state;
    return result;
  }

  void UpdatePlatformAttitude(
      const model::PlatformAttitudeDeg& platform_attitude_deg) override {
    platform_attitude_ = platform_attitude_deg;
  }

  model::PlatformAttitudeDeg GetPlatformAttitude() const override {
    return platform_attitude_;
  }

  void SetControlProfile(const extension::control::RadarControlProfile& control_profile) override {
    control_profile_ = control_profile;
  }

  extension::control::RadarControlProfile GetControlProfile() const override {
    return control_profile_;
  }

  void UpdateConfig(config::PipelineConfig config) override { config_ = config; }

  extension::AssociationQualityMetrics GetLastAssociationQualityMetrics() const override {
    return {};
  }

  extension::SignalPipelineRuntimeState CaptureRuntimeState() const override {
    std::shared_ptr<RuntimeState> state(new RuntimeState());
    state->platform_attitude = platform_attitude_;
    state->control_profile = control_profile_;
    state->config = config_;
    extension::SignalPipelineRuntimeState runtime_state;
    runtime_state.owner_identity = this;
    runtime_state.schema_version = 1U;
    runtime_state.opaque = state;
    return runtime_state;
  }

  void RestoreRuntimeState(const extension::SignalPipelineRuntimeState& state) override {
    if (state.owner_identity != this || state.schema_version != 1U) {
      return;
    }
    const std::shared_ptr<RuntimeState> snapshot =
        std::static_pointer_cast<RuntimeState>(state.opaque);
    if (snapshot == nullptr) {
      return;
    }
    platform_attitude_ = snapshot->platform_attitude;
    control_profile_ = snapshot->control_profile;
    config_ = snapshot->config;
  }

 private:
  struct RuntimeState {
    model::PlatformAttitudeDeg platform_attitude{};
    extension::control::RadarControlProfile control_profile{};
    config::PipelineConfig config{};
  };

  model::PlatformAttitudeDeg platform_attitude_{};
  extension::control::RadarControlProfile control_profile_{};
  config::PipelineConfig config_{};
};

class DummyDecisionEngine : public extension::ITacticalDecisionEngine {
 public:
  extension::TacticalDecisionResult Evaluate(
      const model::DecisionInputFrame& input_frame,
      extension::TacticalStateStore& state_store) override {
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

  airborne_radar::extension::RadarController controller(
      radar_context, signal_pipeline, decision_engine, environment_service);
  controller.UpdateControlReducerConfig({});
  controller.RunOnce();

  return controller.HasLatestTrackOutputFrame() ? 0 : 1;
}
