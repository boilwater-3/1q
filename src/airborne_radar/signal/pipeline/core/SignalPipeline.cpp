#include "airborne_radar/signal/pipeline/core/SignalPipeline.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/pipeline/assembly/RuntimeAssemblySupport.h"
#include "airborne_radar/signal/pipeline/assembly/SignalComponentFactory.h"
#include "airborne_radar/signal/pipeline/core/CycleContextSupport.h"
#include "airborne_radar/signal/pipeline/core/CycleExecutor.h"
#include "airborne_radar/config/mapping/SessionToExecutionMapper.h"
#include "airborne_radar/signal/tracking/IKalmanPredictor.h"
#include "airborne_radar/signal/tracking/IKalmanUpdater.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleManager.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {

void ResetCycleScratch(internal::CycleExecutionScratch* scratch) {
  if (scratch == nullptr) {
    return;
  }
  *scratch = internal::CycleExecutionScratch();
}

bool HasValidEnvironmentCycle(const environment::EnvironmentSnapshot& snapshot) {
  return std::isfinite(snapshot.cycle_dt_sec) != 0 && snapshot.cycle_dt_sec > 0.0f;
}

model::TargetFeature ToModelTargetFeature(const session::RadarSceneTarget& input) {
  model::TargetFeature out;
  out.external_target_id = input.external_target_id;
  out.current_track_velocity_x = input.velocity_x;
  out.current_track_velocity_y = input.velocity_y;
  out.current_track_velocity_z = input.velocity_z;
  out.current_track_speed =
      std::sqrt(input.velocity_x * input.velocity_x + input.velocity_y * input.velocity_y +
                input.velocity_z * input.velocity_z);
  out.current_track_rcs = input.rcs;
  out.range_m = input.range_m;
  out.has_cartesian_position = true;
  out.position_x = input.position_x;
  out.position_y = input.position_y;
  out.position_z = input.position_z;
  out.target_swerling_type = input.target_swerling_type;
  return out;
}

model::TargetFeatureList ToModelTargetFeatureList(const session::RadarSceneTargetList& input) {
  model::TargetFeatureList out;
  out.reserve(input.size());
  for (std::size_t i = 0; i < input.size(); ++i) {
    out.push_back(ToModelTargetFeature(input[i]));
  }
  return out;
}

struct RuntimeConfigState {
  explicit RuntimeConfigState(ExecutionConfig initial_config)
      : base_config(std::move(initial_config)) {}

  ExecutionConfig base_config{};
  extension::control::RadarControlProfile control_profile_{};
};

struct RuntimeOwnedState {
  explicit RuntimeOwnedState(const RuntimeConfigState& config_state)
      : association_engine(assembly::internal::SignalComponentFactory::BuildAssociationConfig(
            config_state.base_config)),
        track_filter(assembly::internal::SignalComponentFactory::BuildTrackFilterConfig(
            config_state.base_config)) {}

  association::DataAssociationEngine association_engine{};
  tracking::TrackFilter track_filter{};
  std::unique_ptr<tracking::IKalmanPredictor> kalman_predictor;
  std::unique_ptr<tracking::IKalmanUpdater> kalman_updater;
  std::unique_ptr<detection::SignalDetector> signal_detector;
  std::unique_ptr<tracking::ITrackLifecycleManager> auto_lifecycle_manager;
};

struct AssociationSeedState {
  std::vector<tracking::AssociationTrackSeed> manual_association_seeds;
  bool has_manual_association_seeds{false};
};

struct SignalPipelineSnapshot {
  ExecutionConfig base_config{};
  extension::control::RadarControlProfile control_profile{};
  AssociationSeedState association_seeds{};
  std::vector<tracking::TrackMeasurement> track_measurements{};
  extension::AssociationQualityMetrics association_quality_metrics{};
  std::uint32_t cycle_index{1U};
  std::uint64_t batch_id{1U};
  association::DataAssociationRuntimeState association_runtime{};
};

struct RuntimeState {
  explicit RuntimeState(ExecutionConfig initial_config)
      : config(std::move(initial_config)), owned(config) {}

  RuntimeConfigState config;
  RuntimeOwnedState owned;
  AssociationSeedState association_seeds;
};

struct CycleState {
  std::uint32_t cycle_index{1};
  std::uint64_t batch_id{1};
  internal::CycleExecutionScratch scratch{};
};

}  // namespace

struct SignalPipeline::Impl {
  explicit Impl(ExecutionConfig initial_config) : runtime_(std::move(initial_config)) {
    RebuildOwnedComponents();
  }

  internal::CycleExecutionRuntime BuildExecutionRuntimeView() {
    return internal::CycleExecutionRuntime(
        runtime_.config.base_config, runtime_.config.control_profile_,
        runtime_.owned.association_engine, runtime_.owned.track_filter,
        *runtime_.owned.auto_lifecycle_manager, runtime_.owned.signal_detector.get(),
        runtime_.association_seeds.manual_association_seeds,
        runtime_.association_seeds.has_manual_association_seeds);
  }

  assembly::internal::ResolvedRuntimePipelineConfig ResolveRuntimeConfig() const {
    return assembly::internal::ResolveRuntimePipelineConfig(runtime_.config.base_config,
                                                            runtime_.config.control_profile_);
  }

  extension::SignalCycleResult RunCycle(const session::RadarSceneTargetList& scene_targets,
                                        const environment::IEnvironmentService& environment) {
    const model::TargetFeatureList input_state = ToModelTargetFeatureList(scene_targets);

    if (runtime_.owned.auto_lifecycle_manager == nullptr) {
      PROJECT_LOG_ERROR(
          "[SignalPipeline] RunCycle aborted because auto_lifecycle_manager is unavailable.");
      ResetCycleScratch(&cycle_.scratch);
      extension::SignalCycleResult result;
      result.abort_reason = extension::SignalCycleAbortReason::kLifecycleUnavailable;
      return result;
    }

    const environment::EnvironmentSnapshot environment_snapshot = environment.SampleEnvironment();
    if (!HasValidEnvironmentCycle(environment_snapshot)) {
      PROJECT_LOG_ERROR(
          "[SignalPipeline] RunCycle aborted because environment cycle is not initialized with a "
          "positive dt_sec.");
      ResetCycleScratch(&cycle_.scratch);
      extension::SignalCycleResult result;
      result.abort_reason = extension::SignalCycleAbortReason::kInvalidEnvironmentCycle;
      return result;
    }
    const internal::CycleExecutionRuntime runtime_execution = BuildExecutionRuntimeView();

    if (!internal::ExecuteCycle(input_state, environment_snapshot, cycle_.cycle_index,
                                cycle_.batch_id, runtime_execution, cycle_.scratch)) {
      ResetCycleScratch(&cycle_.scratch);
      extension::SignalCycleResult result;
      result.abort_reason = extension::SignalCycleAbortReason::kRuntimePreparationFailed;
      return result;
    }

    extension::SignalCycleResult result;
    result.executed_this_cycle = true;
    result.abort_reason = extension::SignalCycleAbortReason::kNone;
    result.updated_scene_targets = scene_targets;
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

  extension::SignalPipelineRuntimeState CaptureRuntimeState() const {
    std::shared_ptr<SignalPipelineSnapshot> snapshot(new SignalPipelineSnapshot());
    snapshot->base_config = runtime_.config.base_config;
    snapshot->control_profile = runtime_.config.control_profile_;
    snapshot->association_seeds = runtime_.association_seeds;
    snapshot->track_measurements = cycle_.scratch.track_measurements;
    snapshot->association_quality_metrics = cycle_.scratch.association_quality_metrics;
    snapshot->cycle_index = cycle_.cycle_index;
    snapshot->batch_id = cycle_.batch_id;
    snapshot->association_runtime = runtime_.owned.association_engine.CaptureRuntimeState();

    extension::SignalPipelineRuntimeState state;
    state.owner_identity = this;
    state.schema_version = 1U;
    state.opaque = snapshot;
    return state;
  }

  void RestoreRuntimeState(const extension::SignalPipelineRuntimeState& state) {
    if (state.owner_identity != this || state.schema_version != 1U) {
      PROJECT_LOG_ERROR(
          "[SignalPipeline] runtime state restore rejected because snapshot owner "
          "or schema does not match this instance.");
      return;
    }
    const std::shared_ptr<SignalPipelineSnapshot> snapshot =
        std::static_pointer_cast<SignalPipelineSnapshot>(state.opaque);
    if (snapshot == nullptr) {
      return;
    }

    runtime_.config.base_config = snapshot->base_config;
    runtime_.config.control_profile_ = snapshot->control_profile;
    RebuildOwnedComponents();
    runtime_.association_seeds = snapshot->association_seeds;
    runtime_.owned.association_engine.RestoreRuntimeState(snapshot->association_runtime);
    cycle_.scratch = internal::CycleExecutionScratch();
    cycle_.scratch.track_measurements = snapshot->track_measurements;
    cycle_.scratch.association_quality_metrics = snapshot->association_quality_metrics;
    cycle_.cycle_index = snapshot->cycle_index;
    cycle_.batch_id = snapshot->batch_id;
  }

  void SetAssociationSeeds(const std::vector<tracking::AssociationTrackSeed>& seeds) {
    if (seeds.empty()) {
      ClearManualAssociationSeeds();
      return;
    }
    if (!runtime_.owned.association_engine.SetAssociationSeeds(seeds)) {
      ClearManualAssociationSeeds();
      return;
    }
    runtime_.association_seeds.has_manual_association_seeds = true;
    runtime_.association_seeds.manual_association_seeds = seeds;
  }

  void ClearManualAssociationSeeds() {
    runtime_.association_seeds.has_manual_association_seeds = false;
    runtime_.association_seeds.manual_association_seeds.clear();
    runtime_.owned.association_engine.ResetAssociationSeedModeToStateless();
  }

  std::unique_ptr<tracking::ITrackLifecycleManager> CreateAutoLifecycleManager() const {
    const assembly::internal::ResolvedRuntimePipelineConfig runtime_config = ResolveRuntimeConfig();
    return assembly::internal::CreateAutoLifecycleManagerForRuntimeConfig(runtime_config.config);
  }

  bool UpdateConfig(ExecutionConfig new_config) {
    const ExecutionConfig previous_config = runtime_.config.base_config;
    runtime_.config.base_config = std::move(new_config);
    if (!internal::SyncAssociationAndTrackFilterConfigs(
            runtime_.config.base_config, &runtime_.owned.association_engine,
            &runtime_.owned.track_filter, runtime_.owned.auto_lifecycle_manager.get())) {
      runtime_.config.base_config = previous_config;
      PROJECT_LOG_ERROR(
          "[SignalPipeline] UpdateConfig rejected because runtime config sync failed; "
          "keeping previous pipeline config.");
      return false;
    }

    assembly::internal::OwnedSignalComponents components =
        assembly::internal::SignalComponentFactory::BuildOwnedPipelineComponents(
            runtime_.config.base_config);
    runtime_.owned.kalman_predictor = std::move(components.kalman_predictor);
    runtime_.owned.kalman_updater = std::move(components.kalman_updater);
    runtime_.owned.signal_detector = std::move(components.signal_detector);
    return true;
  }

  void UpdatePlatformAttitude(const model::PlatformAttitudeDeg& platform_attitude_deg) {
    runtime_.config.base_config.platform_attitude_deg = platform_attitude_deg;
  }

  model::PlatformAttitudeDeg GetPlatformAttitude() const {
    return runtime_.config.base_config.platform_attitude_deg;
  }

  void SetControlProfile(const extension::control::RadarControlProfile& control_profile) {
    runtime_.config.control_profile_ = control_profile;
  }
  extension::control::RadarControlProfile GetControlProfile() const {
    return runtime_.config.control_profile_;
  }

  void RebuildOwnedComponents() {
    assembly::internal::OwnedComponentSlots component_slots;
    component_slots.kalman_predictor = &runtime_.owned.kalman_predictor;
    component_slots.kalman_updater = &runtime_.owned.kalman_updater;
    component_slots.signal_detector = &runtime_.owned.signal_detector;
    component_slots.auto_lifecycle_manager = &runtime_.owned.auto_lifecycle_manager;
    assembly::internal::RebuildOwnedComponentsForPipeline(
        runtime_.config.base_config, runtime_.config.control_profile_, &component_slots);
  }

  RuntimeState runtime_;
  CycleState cycle_;
};

SignalPipeline::SignalPipeline(const ExecutionConfig& config)
  : impl_(std::unique_ptr<Impl>(new Impl(config))) {}

SignalPipeline::SignalPipeline(const session::RadarSessionConfig& config)
  : SignalPipeline(
      ::airborne_radar::config::mapping::MapSessionToExecution(config)) {}

SignalPipeline::~SignalPipeline() = default;

extension::SignalCycleResult SignalPipeline::RunCycle(
    const session::RadarSceneTargetList& scene_targets,
    const environment::IEnvironmentService& environment) {
  return impl_->RunCycle(scene_targets, environment);
}

std::vector<tracking::TrackMeasurement> SignalPipeline::GetLastTrackMeasurements() const {
  return impl_->GetLastTrackMeasurements();
}

extension::AssociationQualityMetrics SignalPipeline::GetLastAssociationQualityMetrics() const {
  return impl_->GetLastAssociationQualityMetrics();
}

extension::SignalPipelineRuntimeState SignalPipeline::CaptureRuntimeState() const {
  return impl_->CaptureRuntimeState();
}

void SignalPipeline::RestoreRuntimeState(const extension::SignalPipelineRuntimeState& state) {
  impl_->RestoreRuntimeState(state);
}

void SignalPipeline::SetAssociationSeeds(const std::vector<tracking::AssociationTrackSeed>& seeds) {
  impl_->SetAssociationSeeds(seeds);
}

void SignalPipeline::ClearManualAssociationSeeds() { impl_->ClearManualAssociationSeeds(); }

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

bool SignalPipeline::UpdateConfig(const session::RadarSessionConfig& config) {
  return UpdateExecutionConfig(
      ::airborne_radar::config::mapping::MapSessionToExecution(config));
}

bool SignalPipeline::UpdateExecutionConfig(const ExecutionConfig& config) {
  return impl_->UpdateConfig(config);
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
