#include "airborne_radar/signal/runtime/RuntimeAssemblySupport.h"

#include <utility>

#include "airborne_radar/signal/pipeline/ControlProfileEffects.h"
#include "airborne_radar/signal/runtime/SignalComponentFactory.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace runtime {
namespace internal {

namespace {

LifecycleAssemblyArtifacts BuildGuaranteedLifecycleAssemblyArtifacts(
    const ResolvedRuntimeSignalPipelineConfig& resolved_config) {
  LifecycleAssemblyArtifacts artifacts = SignalComponentFactory::BuildLifecycleAssemblyArtifacts(
      resolved_config.public_config, resolved_config.internal_config);
  if (artifacts.lifecycle_manager != nullptr) {
    return artifacts;
  }

  ResolvedRuntimeSignalPipelineConfig fallback_config = resolved_config;
  fallback_config.public_config.lifecycle.enable_imm_lifecycle = false;
  PROJECT_LOG_ERROR(
      "[SignalPipeline] lifecycle auto-assembly produced null manager; "
      "falling back to non-IMM lifecycle manager.");
  return SignalComponentFactory::BuildLifecycleAssemblyArtifacts(
      fallback_config.public_config, fallback_config.internal_config);
}

class AutoConfiguredLifecycleManager final : public tracking::ITrackLifecycleManager {
 public:
  explicit AutoConfiguredLifecycleManager(const ResolvedRuntimeSignalPipelineConfig& resolved_config)
      : assembly_(BuildGuaranteedLifecycleAssemblyArtifacts(resolved_config)) {}

  void Update(const tracking::CycleContext& cycle,
              const std::vector<tracking::TrackMeasurement>& measurements) override {
    if (assembly_.lifecycle_manager == nullptr) {
      return;
    }
    assembly_.lifecycle_manager->Update(cycle, measurements);
  }

  common::model::TargetFeatureList BuildFeatureSnapshot() const override {
    if (assembly_.lifecycle_manager == nullptr) {
      return common::model::TargetFeatureList();
    }
    return assembly_.lifecycle_manager->BuildFeatureSnapshot();
  }

  common::model::DecisionTrackSnapshotList BuildDecisionSnapshot() const override {
    if (assembly_.lifecycle_manager == nullptr) {
      return common::model::DecisionTrackSnapshotList();
    }
    return assembly_.lifecycle_manager->BuildDecisionSnapshot();
  }

  common::model::DecisionInputFrame BuildDecisionFrame(std::uint32_t cycle_index, std::uint64_t batch_id,
                                                bool environment_jamming_detected) const override {
    if (assembly_.lifecycle_manager == nullptr) {
      common::model::DecisionInputFrame frame;
      frame.cycle_index = cycle_index;
      frame.batch_id = batch_id;
      frame.environment_jamming_detected = environment_jamming_detected;
      return frame;
    }
    return assembly_.lifecycle_manager->BuildDecisionFrame(cycle_index, batch_id,
                                                           environment_jamming_detected);
  }

  std::vector<tracking::AssociationTrackSeed> BuildAssociationSeeds() const override {
    if (assembly_.lifecycle_manager == nullptr) {
      return std::vector<tracking::AssociationTrackSeed>();
    }
    return assembly_.lifecycle_manager->BuildAssociationSeeds();
  }

 private:
  LifecycleAssemblyArtifacts assembly_;
};

/**
 * @brief 检查自有组件槽位是否全部持有有效指针。
 * @param slots  待检查的组件槽位集合。
 * @return 所有必需组件均非空时返回 true，否则返回 false。
 */
bool HasValidOwnedComponentSlots(const OwnedComponentSlots& slots) {
  return slots.kalman_predictor != nullptr && slots.kalman_updater != nullptr &&
         slots.signal_detector != nullptr && slots.auto_lifecycle_manager != nullptr;
}

}  // namespace

ResolvedRuntimeSignalPipelineConfig BuildRuntimeConfigFromControlProfile(
    const SignalPipelineConfig& base_config,
    const pipeline::internal::InternalSignalPipelineConfig& base_internal_config,
    const common::control::RadarControlProfile& control_profile) {
  ResolvedRuntimeSignalPipelineConfig resolved_config;
  resolved_config.public_config = base_config;
  resolved_config.internal_config = base_internal_config;
  pipeline::internal::ApplyControlProfileToConfig(control_profile, &resolved_config.public_config,
                                                  &resolved_config.internal_config);
  return resolved_config;
}

std::unique_ptr<tracking::ITrackLifecycleManager> CreateAutoLifecycleManagerForRuntimeConfig(
    const SignalPipelineConfig& runtime_config) {
  return CreateAutoLifecycleManagerForRuntimeConfig(
      runtime_config, pipeline::internal::BuildInternalSignalPipelineConfig(runtime_config));
}

std::unique_ptr<tracking::ITrackLifecycleManager> CreateAutoLifecycleManagerForRuntimeConfig(
    const SignalPipelineConfig& runtime_config,
    const pipeline::internal::InternalSignalPipelineConfig& internal_config) {
  if (!runtime_config.lifecycle.enable_auto_lifecycle_manager) {
    return std::unique_ptr<tracking::ITrackLifecycleManager>();
  }
  ResolvedRuntimeSignalPipelineConfig resolved_config;
  resolved_config.public_config = runtime_config;
  resolved_config.internal_config = internal_config;
  return std::unique_ptr<tracking::ITrackLifecycleManager>(new AutoConfiguredLifecycleManager(
      resolved_config));
}

void RebuildOwnedComponentsForPipeline(
    const SignalPipelineConfig& base_config,
    const pipeline::internal::InternalSignalPipelineConfig& base_internal_config,
    const common::control::RadarControlProfile& control_profile,
    OwnedComponentSlots* slots) {
  if (slots == nullptr || !HasValidOwnedComponentSlots(*slots)) {
    return;
  }

  OwnedSignalComponents components =
      SignalComponentFactory::BuildOwnedPipelineComponents(base_config, base_internal_config);
  *slots->kalman_predictor = std::move(components.kalman_predictor);
  *slots->kalman_updater = std::move(components.kalman_updater);
  *slots->signal_detector = std::move(components.signal_detector);

  const ResolvedRuntimeSignalPipelineConfig resolved_runtime_config =
      BuildRuntimeConfigFromControlProfile(base_config, base_internal_config, control_profile);
  *slots->auto_lifecycle_manager = CreateAutoLifecycleManagerForRuntimeConfig(
      resolved_runtime_config.public_config, resolved_runtime_config.internal_config);
}

}  // namespace internal
}  // namespace runtime
}  // namespace signal
}  // namespace airborne_radar
