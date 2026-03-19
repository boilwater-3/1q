/**
 * @file advanced_extension_consumer.cpp
 * @brief 验证安装后的高级扩展接口可被外部工程实现并接入控制器。
 */

#include <vector>

#include "1q/airborne_radar/common/TrackOutputFrame.h"
#include "1q/airborne_radar/core/context/IRadarContext.h"
#include "1q/airborne_radar/core/controller/RadarController.h"
#include "1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/signal/pipeline/ISignalPipeline.h"

namespace airborne_radar {
namespace {

class DummyRadarContext : public core::context::IRadarContext {
 public:
  common::TargetFeatureList GetTargetFeatures() const override { return {}; }

  common::PlatformAttitudeDeg GetPlatformAttitude() const override {
    return platform_attitude_;
  }

  float GetCycleDeltaTimeSec() const override { return 1.0f; }

  void SubmitControlCommand(common::RadarCommand cmd) override {
    commands_.push_back(cmd);
  }

  void UpdateRadarControlProfile(
      const common::RadarControlProfile& profile) override {
    control_profile_ = profile;
  }

 private:
  common::PlatformAttitudeDeg platform_attitude_{};
  common::RadarControlProfile control_profile_{};
  std::vector<common::RadarCommand> commands_{};
};

class DummyEnvironmentService : public environment::IEnvironmentService {
 public:
  void BeginCycle(const environment::EnvironmentCycleContext& cycle_context)
      override {
    cycle_context_ = cycle_context;
  }

  environment::EnvironmentSnapshot SampleEnvironment() const override {
    environment::EnvironmentSnapshot snapshot;
    snapshot.cycle_dt_sec = cycle_context_.dt_sec;
    return snapshot;
  }

 private:
  environment::EnvironmentCycleContext cycle_context_{};
};

class DummySignalPipeline : public signal::pipeline::ISignalPipeline {
 public:
  signal::pipeline::SignalCycleResult RunCycle(
      const common::TargetFeatureList& input_state,
      const environment::IEnvironmentService& environment) override {
    (void)environment;
    signal::pipeline::SignalCycleResult result;
    result.updated_features = input_state;
    return result;
  }

  void UpdatePlatformAttitude(
      const common::PlatformAttitudeDeg& platform_attitude_deg) override {
    platform_attitude_ = platform_attitude_deg;
  }

  common::PlatformAttitudeDeg GetPlatformAttitude() const override {
    return platform_attitude_;
  }

  void SetControlProfile(
      const common::RadarControlProfile& control_profile) override {
    control_profile_ = control_profile;
  }

  common::RadarControlProfile GetControlProfile() const override {
    return control_profile_;
  }

 private:
  common::PlatformAttitudeDeg platform_attitude_{};
  common::RadarControlProfile control_profile_{};
};

class DummyDecisionEngine : public decision::pipeline::ITacticalDecisionEngine {
 public:
  decision::pipeline::TacticalDecisionResult Evaluate(
      const common::DecisionInputFrame& input_frame,
      decision::pipeline::TacticalStateStore& state_store) override {
    (void)input_frame;
    (void)state_store;
    return {};
  }
};

} // namespace
} // namespace airborne_radar

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
