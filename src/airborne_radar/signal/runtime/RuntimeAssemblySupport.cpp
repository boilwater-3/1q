#include "airborne_radar/signal/runtime/RuntimeAssemblySupport.h"

#include <utility>

#include "airborne_radar/signal/pipeline/ControlProfileEffects.h"
#include "airborne_radar/signal/runtime/SignalComponentFactory.h"

namespace airborne_radar {
namespace signal {
namespace runtime {
namespace internal {

namespace {

class AutoConfiguredLifecycleManager final : public tracking::ITrackLifecycleManager {
 public:
  explicit AutoConfiguredLifecycleManager(const SignalPipelineConfig& config)
      : assembly_(SignalComponentFactory::BuildLifecycleAssemblyArtifacts(config)) {}

  void Update(const tracking::CycleContext& cycle,
              const std::vector<tracking::TrackMeasurement>& measurements) override {
    assembly_.lifecycle_manager->Update(cycle, measurements);
  }

  common::TargetFeatureList BuildFeatureSnapshot() const override {
    return assembly_.lifecycle_manager->BuildFeatureSnapshot();
  }

  common::DecisionTrackSnapshotList BuildDecisionSnapshot() const override {
    return assembly_.lifecycle_manager->BuildDecisionSnapshot();
  }

  common::DecisionInputFrame BuildDecisionFrame(std::uint32_t cycle_index, std::uint64_t batch_id,
                                                bool environment_jamming_detected) const override {
    return assembly_.lifecycle_manager->BuildDecisionFrame(cycle_index, batch_id,
                                                           environment_jamming_detected);
  }

  std::vector<tracking::AssociationTrackSeed> BuildAssociationSeeds() const override {
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

SignalPipelineConfig BuildRuntimeConfigFromControlProfile(
    const SignalPipelineConfig& base_config, const common::RadarControlProfile& control_profile) {
  SignalPipelineConfig runtime_config = base_config;
  pipeline::internal::ApplyControlProfileToConfig(control_profile, &runtime_config);
  return runtime_config;
}

std::unique_ptr<tracking::ITrackLifecycleManager> CreateAutoLifecycleManagerForRuntimeConfig(
    const SignalPipelineConfig& runtime_config) {
  if (!runtime_config.lifecycle.enable_auto_lifecycle_manager) {
    return std::unique_ptr<tracking::ITrackLifecycleManager>();
  }
  return std::unique_ptr<tracking::ITrackLifecycleManager>(
      new AutoConfiguredLifecycleManager(runtime_config));
}

void RebuildOwnedComponentsForPipeline(const SignalPipelineConfig& base_config,
                                       const common::RadarControlProfile& control_profile,
                                       OwnedComponentSlots* slots) {
  if (slots == nullptr || !HasValidOwnedComponentSlots(*slots)) {
    return;
  }

  OwnedSignalComponents components =
      SignalComponentFactory::BuildOwnedPipelineComponents(base_config);
  *slots->kalman_predictor = std::move(components.kalman_predictor);
  *slots->kalman_updater = std::move(components.kalman_updater);
  *slots->signal_detector = std::move(components.signal_detector);

  const SignalPipelineConfig runtime_config =
      BuildRuntimeConfigFromControlProfile(base_config, control_profile);
  *slots->auto_lifecycle_manager = CreateAutoLifecycleManagerForRuntimeConfig(runtime_config);
}

}  // namespace internal
}  // namespace runtime
}  // namespace signal
}  // namespace airborne_radar
