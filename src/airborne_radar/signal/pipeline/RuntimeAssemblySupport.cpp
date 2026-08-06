#include "airborne_radar/signal/pipeline/RuntimeAssemblySupport.h"

#include <utility>

#include "airborne_radar/config/SignalEngineeringConfig.h"
#include "airborne_radar/signal/pipeline/ControlProfileEffects.h"
#include "airborne_radar/signal/pipeline/ImmMatrixDefaults.h"
#include "airborne_radar/signal/pipeline/SignalComponentFactory.h"
#include "airborne_radar/signal/tracking/TrackLifecycleManager.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {

struct LifecycleConfigSignature {
  bool enable_imm_lifecycle{false};
  std::size_t imm_model_count{0U};
  tracking::TrackPoolThreadSafetyMode track_pool_thread_safety_mode{
      tracking::TrackPoolThreadSafetyMode::kSingleThreadNoLock};
};

// Thin wrappers over the shared imm_defaults unit. The runtime rebuild path
// keeps its original silent behavior by passing a null violation reporter.
Eigen::MatrixXf BuildImmTransitionProbabilityOrDefault(const ExecutionConfig& config,
                                                       std::size_t model_count) {
  return imm_defaults::BuildTransitionProbability(config, model_count, /*report_violation=*/{});
}

Eigen::VectorXf BuildImmInitialWeightsOrDefault(const ExecutionConfig& config,
                                                std::size_t model_count) {
  return imm_defaults::BuildInitialWeights(config, model_count, /*report_violation=*/{});
}

LifecycleConfigSignature BuildLifecycleConfigSignature(const ExecutionConfig& config) {
  LifecycleConfigSignature signature;
  signature.enable_imm_lifecycle = config.lifecycle.engineering.enable_imm_lifecycle;
  signature.imm_model_count = config.lifecycle.imm_model_noise_diff_coeffs.size();
  signature.track_pool_thread_safety_mode = config.lifecycle.track_pool_thread_safety_mode;
  return signature;
}

bool ShouldRebuildLifecycleAssembly(const LifecycleConfigSignature& before,
                                    const LifecycleConfigSignature& after) {
  return before.enable_imm_lifecycle != after.enable_imm_lifecycle ||
         before.imm_model_count != after.imm_model_count ||
         before.track_pool_thread_safety_mode != after.track_pool_thread_safety_mode;
}

void AssignLifecycleSignature(const LifecycleConfigSignature& source,
                              LifecycleConfigSignature* destination) {
  if (destination == nullptr) {
    return;
  }
  destination->enable_imm_lifecycle = source.enable_imm_lifecycle;
  destination->imm_model_count = source.imm_model_count;
  destination->track_pool_thread_safety_mode = source.track_pool_thread_safety_mode;
}

LifecycleAssemblyArtifacts BuildLifecycleAssemblyArtifactsOrLogFailure(
    const ExecutionConfig& config) {
  LifecycleAssemblyArtifacts artifacts =
      SignalComponentFactory::BuildLifecycleAssemblyArtifacts(config);
  if (artifacts.lifecycle_manager == nullptr) {
    // 中译：生命周期管理器自动装配失败（当前运行配置），未创建管理器。
    // 标识：装配失败——生命周期管理不可用，后续周期将中止或降级。
    PROJECT_LOG_ERROR(
        "[SignalPipeline] lifecycle auto-assembly failed for current runtime config; "
        "no lifecycle manager created.");
  }
  return artifacts;
}

class AutoConfiguredLifecycleManager final : public tracking::ITrackLifecycleManager {
 public:
  explicit AutoConfiguredLifecycleManager(const ExecutionConfig& config,
                                          LifecycleAssemblyArtifacts assembly)
      : config_(config),
        signature_(BuildLifecycleConfigSignature(config_)),
        assembly_(std::move(assembly)) {}

  void Update(const tracking::CycleContext& cycle,
              const std::vector<tracking::TrackMeasurement>& measurements) override {
    if (assembly_.lifecycle_manager == nullptr) {
      return;
    }
    assembly_.lifecycle_manager->Update(cycle, measurements);
  }

  session::ArSceneTargetList BuildSceneTargetSnapshot() const override {
    if (assembly_.lifecycle_manager == nullptr) {
      return session::ArSceneTargetList();
    }
    return assembly_.lifecycle_manager->BuildSceneTargetSnapshot();
  }

  session::TrackStateSnapshotList BuildTrackStateSnapshots() const override {
    if (assembly_.lifecycle_manager == nullptr) {
      return session::TrackStateSnapshotList();
    }
    return assembly_.lifecycle_manager->BuildTrackStateSnapshots();
  }

  std::vector<tracking::AssociationTrackSeed> BuildAssociationSeeds() const override {
    if (assembly_.lifecycle_manager == nullptr) {
      return std::vector<tracking::AssociationTrackSeed>();
    }
    return assembly_.lifecycle_manager->BuildAssociationSeeds();
  }

  tracking::TrackLifecycleRuntimeState CaptureRuntimeState() const override {
    if (assembly_.lifecycle_manager == nullptr) {
      return tracking::TrackLifecycleRuntimeState();
    }
    return assembly_.lifecycle_manager->CaptureRuntimeState();
  }

  void RestoreRuntimeState(const tracking::TrackLifecycleRuntimeState& state) override {
    if (assembly_.lifecycle_manager == nullptr) {
      return;
    }
    assembly_.lifecycle_manager->RestoreRuntimeState(state);
  }

  void SyncRuntimeTuning(const tracking::LifecycleConfig& lifecycle_config,
                         float kalman_noise_diff_coeff, float kalman_measurement_noise_std,
                         const std::vector<float>& imm_model_noise_diff_coeffs,
                         const Eigen::MatrixXf& imm_transition_probability,
                         const Eigen::VectorXf& imm_initial_weights) override {
    if (assembly_.lifecycle_manager == nullptr) {
      return;
    }
    tracking::TrackLifecycleManager* concrete_manager =
        dynamic_cast<tracking::TrackLifecycleManager*>(assembly_.lifecycle_manager.get());
    if (concrete_manager == nullptr) {
      return;
    }
    concrete_manager->SyncRuntimeTuning(lifecycle_config, kalman_noise_diff_coeff,
                                        kalman_measurement_noise_std, imm_model_noise_diff_coeffs,
                                        imm_transition_probability, imm_initial_weights);
  }

  bool SyncRuntimeConfig(const ExecutionConfig& config) {
    const LifecycleConfigSignature incoming_signature = BuildLifecycleConfigSignature(config);
    if (ShouldRebuildLifecycleAssembly(signature_, incoming_signature)) {
      // 中译：运行期生命周期配置拓扑发生变化，正在重建装配并重置生命周期状态。
      // 标识：运行期配置变更——拓扑签名变化触发重建，重建期间生命周期状态归零。
      PROJECT_LOG_WARN(
          "[SignalPipeline] lifecycle config topology changed during runtime; rebuilding "
          "auto-lifecycle assembly and resetting lifecycle state.");
      LifecycleAssemblyArtifacts rebuilt_assembly =
          BuildLifecycleAssemblyArtifactsOrLogFailure(config);
      if (rebuilt_assembly.lifecycle_manager == nullptr) {
        // 中译：生命周期拓扑重建失败，保留上一次装配。
        // 标识：重建失败回退——新配置无法装配时沿用旧配置，同步返回失败。
        PROJECT_LOG_ERROR(
            "[SignalPipeline] lifecycle topology rebuild failed; keeping previous "
            "lifecycle assembly.");
        return false;
      }
      config_ = config;
      AssignLifecycleSignature(incoming_signature, &signature_);
      assembly_ = std::move(rebuilt_assembly);
      return true;
    }

    config_ = config;
    AssignLifecycleSignature(incoming_signature, &signature_);
    const tracking::LifecycleConfig lifecycle_config =
        SignalComponentFactory::BuildLifecycleConfig(config_);
    const float kalman_noise_diff_coeff = config_.tracking.kalman_noise_diff_coeff;
    const float kalman_measurement_noise_std =
        config_.tracking.engineering.kalman_measurement_noise_std;
    const std::vector<float>& imm_model_noise_diff_coeffs =
        config_.lifecycle.imm_model_noise_diff_coeffs;
    const Eigen::MatrixXf imm_transition_probability =
        BuildImmTransitionProbabilityOrDefault(config_, imm_model_noise_diff_coeffs.size());
    const Eigen::VectorXf imm_initial_weights =
        BuildImmInitialWeightsOrDefault(config_, imm_model_noise_diff_coeffs.size());
    SyncRuntimeTuning(lifecycle_config, kalman_noise_diff_coeff, kalman_measurement_noise_std,
                      imm_model_noise_diff_coeffs, imm_transition_probability, imm_initial_weights);
    return true;
  }

 private:
  ExecutionConfig config_;
  LifecycleConfigSignature signature_;
  LifecycleAssemblyArtifacts assembly_;
};

bool HasValidOwnedComponentSlots(const OwnedComponentSlots& slots) {
  return slots.kalman_predictor != nullptr && slots.kalman_updater != nullptr &&
         slots.signal_detector != nullptr && slots.auto_lifecycle_manager != nullptr;
}

}  // namespace

ResolvedRuntimePipelineConfig ResolveRuntimePipelineConfig(
    const ExecutionConfig& base_config,
    const session::ArControlProfile& control_profile) {
  ResolvedRuntimePipelineConfig resolved;
  resolved.config = base_config;
  ApplyControlProfileToConfig(control_profile, &resolved.config);
  return resolved;
}

std::unique_ptr<tracking::ITrackLifecycleManager> CreateAutoLifecycleManagerForRuntimeConfig(
    const ExecutionConfig& runtime_config) {
  LifecycleAssemblyArtifacts assembly = BuildLifecycleAssemblyArtifactsOrLogFailure(runtime_config);
  if (assembly.lifecycle_manager == nullptr) {
    return std::unique_ptr<tracking::ITrackLifecycleManager>();
  }
  return std::unique_ptr<tracking::ITrackLifecycleManager>(
      new AutoConfiguredLifecycleManager(runtime_config, std::move(assembly)));
}

void RebuildOwnedComponentsForPipeline(
    const ExecutionConfig& base_config,
    const session::ArControlProfile& control_profile, OwnedComponentSlots* slots) {
  if (slots == nullptr || !HasValidOwnedComponentSlots(*slots)) {
    return;
  }

  OwnedSignalComponents components =
      SignalComponentFactory::BuildOwnedPipelineComponents(base_config);
  *slots->kalman_predictor = std::move(components.kalman_predictor);
  *slots->kalman_updater = std::move(components.kalman_updater);
  *slots->signal_detector = std::move(components.signal_detector);

  const ResolvedRuntimePipelineConfig resolved_runtime_config =
      ResolveRuntimePipelineConfig(base_config, control_profile);
  *slots->auto_lifecycle_manager =
      CreateAutoLifecycleManagerForRuntimeConfig(resolved_runtime_config.config);
}

bool SyncAutoLifecycleManagerForResolvedRuntimeConfig(
    const ResolvedRuntimePipelineConfig& resolved_runtime_config,
    tracking::ITrackLifecycleManager* auto_lifecycle_manager) {
  AutoConfiguredLifecycleManager* auto_configured_manager =
      dynamic_cast<AutoConfiguredLifecycleManager*>(auto_lifecycle_manager);
  if (auto_configured_manager == nullptr) {
    return false;
  }
  return auto_configured_manager->SyncRuntimeConfig(resolved_runtime_config.config);
}

bool SyncAutoLifecycleManagerForRuntimeConfig(
    const ExecutionConfig& runtime_config,
    tracking::ITrackLifecycleManager* auto_lifecycle_manager) {
  ResolvedRuntimePipelineConfig resolved;
  resolved.config = runtime_config;
  return SyncAutoLifecycleManagerForResolvedRuntimeConfig(resolved, auto_lifecycle_manager);
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
