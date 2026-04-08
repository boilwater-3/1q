#include "airborne_radar/signal/pipeline/core/SignalPipeline.h"

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "1q/airborne_radar/extension/IEnvironmentService.h"
#include "airborne_radar/signal/assembly/DataOutputManager.h"
#include "airborne_radar/signal/assembly/IDataOutputManager.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/pipeline/core/CycleContextSupport.h"
#include "airborne_radar/signal/pipeline/core/CycleExecutor.h"
#include "airborne_radar/signal/pipeline/assembly/RuntimeAssemblySupport.h"
#include "airborne_radar/signal/pipeline/assembly/SignalComponentFactory.h"
#include "airborne_radar/signal/tracking/IKalmanPredictor.h"
#include "airborne_radar/signal/tracking/IKalmanUpdater.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleManager.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {
struct RuntimeState {
  explicit RuntimeState(SignalPipelineConfig initial_config)
      : config(std::move(initial_config)),
        internal_config(internal::BuildInternalSignalPipelineConfig(config)),
        association_engine(assembly::internal::SignalComponentFactory::BuildAssociationConfig(
            config, internal_config)),
        track_filter(
            assembly::internal::SignalComponentFactory::BuildTrackFilterConfig(internal_config)),
        output_manager(std::unique_ptr<signal::assembly::IDataOutputManager>(
            new signal::assembly::DataOutputManager())) {}

  SignalPipelineConfig config{};
  internal::InternalSignalPipelineConfig internal_config{};
  extension::control::RadarControlProfile control_profile_{};
  association::DataAssociationEngine association_engine{};
  tracking::TrackFilter track_filter{};
  std::unique_ptr<signal::assembly::IDataOutputManager> output_manager;
  std::unique_ptr<tracking::IKalmanPredictor> kalman_predictor;
  std::unique_ptr<tracking::IKalmanUpdater> kalman_updater;
  std::unique_ptr<detection::SignalDetector> signal_detector;
  std::unique_ptr<tracking::ITrackLifecycleManager> auto_lifecycle_manager;
  std::vector<tracking::AssociationTrackSeed> manual_association_seeds;
  bool has_manual_association_seeds{false};
};

struct CycleState {
  std::uint32_t cycle_index{1};
  std::uint64_t batch_id{1};
  internal::CycleExecutionScratch scratch{};
};

}  // namespace

struct SignalPipeline::Impl {
  explicit Impl(SignalPipelineConfig initial_config) : runtime_(std::move(initial_config)) {
    RebuildOwnedComponents();
  }

  extension::SignalCycleResult RunCycle(
      const model::TargetFeatureList& input_state,
      const extension::IEnvironmentService& environment) {
    internal::CycleExecutionRuntime runtime_execution;
    runtime_execution.base_config = &runtime_.config;
    runtime_execution.base_internal_config = &runtime_.internal_config;
    runtime_execution.control_profile = &runtime_.control_profile_;
    runtime_execution.association_engine = &runtime_.association_engine;
    runtime_execution.track_filter = &runtime_.track_filter;
    runtime_execution.signal_detector = runtime_.signal_detector.get();
    runtime_execution.output_manager = runtime_.output_manager.get();
    runtime_execution.auto_lifecycle_manager = runtime_.auto_lifecycle_manager.get();
    runtime_execution.manual_association_seeds = &runtime_.manual_association_seeds;
    runtime_execution.has_manual_association_seeds = runtime_.has_manual_association_seeds;

    internal::ExecuteCycle(input_state, environment, cycle_.cycle_index, cycle_.batch_id,
                           runtime_execution, &cycle_.scratch);

    extension::SignalCycleResult result;
    result.updated_features = cycle_.scratch.output_state;
    result.decision_frame = cycle_.scratch.decision_frame;
    result.association_quality_metrics = cycle_.scratch.association_quality_metrics;
    ++cycle_.cycle_index;
    ++cycle_.batch_id;
    return result;
  }

  std::vector<tracking::TrackMeasurement> GetLastTrackMeasurements() const {
    return cycle_.scratch.track_measurements;
  }

  extension::AssociationQualityMetrics GetLastAssociationQualityMetrics() const {
    return cycle_.scratch.association_quality_metrics;
  }

  void SetAssociationSeeds(const std::vector<tracking::AssociationTrackSeed>& seeds) {
    runtime_.has_manual_association_seeds = true;
    runtime_.manual_association_seeds = seeds;
    runtime_.association_engine.SetAssociationSeeds(runtime_.manual_association_seeds);
  }

  void ResetAssociationSeedModeToStateless() {
    runtime_.has_manual_association_seeds = false;
    runtime_.manual_association_seeds.clear();
    runtime_.association_engine.ResetAssociationSeedModeToStateless();
  }

  std::unique_ptr<tracking::ITrackLifecycleManager> CreateAutoLifecycleManager() const {
    const assembly::internal::ResolvedRuntimeSignalPipelineConfig runtime_config =
        BuildRuntimeConfig();
    return assembly::internal::CreateAutoLifecycleManagerForRuntimeConfig(
        runtime_config.public_config, runtime_config.internal_config);
  }

  void UpdateConfig(SignalPipelineConfig new_config) {
    runtime_.config = std::move(new_config);
    runtime_.internal_config = internal::BuildInternalSignalPipelineConfig(runtime_.config);
    internal::SyncAssociationAndTrackFilterConfigs(runtime_.config, runtime_.internal_config,
                                                   &runtime_.association_engine,
                                                   &runtime_.track_filter);
    RebuildOwnedComponents();
  }

  void UpdatePlatformAttitude(const model::PlatformAttitudeDeg& platform_attitude_deg) {
    runtime_.config.beam_control.platform_attitude_deg = platform_attitude_deg;
  }

  model::PlatformAttitudeDeg GetPlatformAttitude() const {
    return runtime_.config.beam_control.platform_attitude_deg;
  }

  assembly::internal::ResolvedRuntimeSignalPipelineConfig BuildRuntimeConfig() const {
    return assembly::internal::BuildRuntimeConfigFromControlProfile(
        runtime_.config, runtime_.internal_config, runtime_.control_profile_);
  }

  void SetControlProfile(const extension::control::RadarControlProfile& control_profile) {
    runtime_.control_profile_ = control_profile;
  }
  extension::control::RadarControlProfile GetControlProfile() const {
    return runtime_.control_profile_;
  }

  void RebuildOwnedComponents() {
    assembly::internal::OwnedComponentSlots component_slots;
    component_slots.kalman_predictor = &runtime_.kalman_predictor;
    component_slots.kalman_updater = &runtime_.kalman_updater;
    component_slots.signal_detector = &runtime_.signal_detector;
    component_slots.auto_lifecycle_manager = &runtime_.auto_lifecycle_manager;
    assembly::internal::RebuildOwnedComponentsForPipeline(
        runtime_.config, runtime_.internal_config, runtime_.control_profile_, &component_slots);
  }

  RuntimeState runtime_;
  CycleState cycle_;
};

SignalPipeline::SignalPipeline(SignalPipelineConfig config)
    : impl_(std::unique_ptr<Impl>(new Impl(std::move(config)))) {}

SignalPipeline::~SignalPipeline() = default;

extension::SignalCycleResult SignalPipeline::RunCycle(
    const model::TargetFeatureList& input_state,
    const extension::IEnvironmentService& environment) {
  return impl_->RunCycle(input_state, environment);
}

std::vector<tracking::TrackMeasurement> SignalPipeline::GetLastTrackMeasurements() const {
  return impl_->GetLastTrackMeasurements();
}

extension::AssociationQualityMetrics SignalPipeline::GetLastAssociationQualityMetrics() const {
  return impl_->GetLastAssociationQualityMetrics();
}

void SignalPipeline::SetAssociationSeeds(const std::vector<tracking::AssociationTrackSeed>& seeds) {
  impl_->SetAssociationSeeds(seeds);
}

void SignalPipeline::ResetAssociationSeedModeToStateless() {
  impl_->ResetAssociationSeedModeToStateless();
}

std::unique_ptr<tracking::ITrackLifecycleManager> SignalPipeline::CreateAutoLifecycleManager()
    const {
  return impl_->CreateAutoLifecycleManager();
}

void SignalPipeline::UpdatePlatformAttitude(
    const model::PlatformAttitudeDeg& platform_attitude_deg) {
  impl_->UpdatePlatformAttitude(platform_attitude_deg);
}

model::PlatformAttitudeDeg SignalPipeline::GetPlatformAttitude() const {
  return impl_->GetPlatformAttitude();
}

void SignalPipeline::SetControlProfile(
    const extension::control::RadarControlProfile& control_profile) {
  impl_->SetControlProfile(control_profile);
}

extension::control::RadarControlProfile SignalPipeline::GetControlProfile() const {
  return impl_->GetControlProfile();
}

void SignalPipeline::UpdateConfig(SignalPipelineConfig config) {
  impl_->UpdateConfig(std::move(config));
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
