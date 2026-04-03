#include "airborne_radar/signal/runtime/RuntimeAssemblySupport.h"

#include <cmath>
#include <utility>

#include "airborne_radar/signal/pipeline/ControlProfileEffects.h"
#include "airborne_radar/signal/runtime/SignalComponentFactory.h"
#include "airborne_radar/signal/tracking/TrackLifecycleManager.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace runtime {
namespace internal {

namespace {

struct LifecycleConfigSignature {
  bool enable_imm_lifecycle{false};
  common::config::KalmanUpdateBackend kalman_update_backend{
      common::config::KalmanUpdateBackend::kStandardKfJoseph};
  std::size_t imm_model_count{0U};
  tracking::TrackPoolThreadSafetyMode track_pool_thread_safety_mode{
      tracking::TrackPoolThreadSafetyMode::kSingleThreadNoLock};
};

bool IsNearOne(float value) { return std::fabs(value - 1.0f) <= 1.0e-3f; }

Eigen::MatrixXf BuildImmTransitionProbabilityOrDefault(
    const pipeline::internal::InternalSignalPipelineConfig& internal_config,
    std::size_t model_count) {
  if (model_count == 0U) {
    return Eigen::MatrixXf();
  }
  if (internal_config.lifecycle.imm_transition_probability.empty()) {
    Eigen::MatrixXf matrix = Eigen::MatrixXf::Constant(
        static_cast<Eigen::Index>(model_count), static_cast<Eigen::Index>(model_count),
        model_count > 1U ? 0.05f / static_cast<float>(model_count - 1U) : 1.0f);
    matrix.diagonal().setConstant(model_count > 1U ? 0.95f : 1.0f);
    return matrix;
  }
  if (internal_config.lifecycle.imm_transition_probability.size() != model_count * model_count) {
    return Eigen::MatrixXf();
  }
  Eigen::MatrixXf matrix(static_cast<Eigen::Index>(model_count),
                         static_cast<Eigen::Index>(model_count));
  for (std::size_t r = 0; r < model_count; ++r) {
    float row_sum = 0.0f;
    for (std::size_t c = 0; c < model_count; ++c) {
      const float value = internal_config.lifecycle.imm_transition_probability[r * model_count + c];
      if (std::isfinite(value) == 0 || value < 0.0f || value > 1.0f) {
        return Eigen::MatrixXf();
      }
      matrix(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = value;
      row_sum += value;
    }
    if (!IsNearOne(row_sum)) {
      return Eigen::MatrixXf();
    }
  }
  return matrix;
}

Eigen::VectorXf BuildImmInitialWeightsOrDefault(
    const pipeline::internal::InternalSignalPipelineConfig& internal_config,
    std::size_t model_count) {
  if (model_count == 0U) {
    return Eigen::VectorXf();
  }
  if (internal_config.lifecycle.imm_initial_weights.empty()) {
    return Eigen::VectorXf::Constant(static_cast<Eigen::Index>(model_count),
                                     1.0f / static_cast<float>(model_count));
  }
  if (internal_config.lifecycle.imm_initial_weights.size() != model_count) {
    return Eigen::VectorXf();
  }
  Eigen::VectorXf weights(static_cast<Eigen::Index>(model_count));
  float sum = 0.0f;
  for (std::size_t i = 0; i < model_count; ++i) {
    const float value = internal_config.lifecycle.imm_initial_weights[i];
    if (std::isfinite(value) == 0 || value < 0.0f || value > 1.0f) {
      return Eigen::VectorXf();
    }
    weights(static_cast<Eigen::Index>(i)) = value;
    sum += value;
  }
  if (!IsNearOne(sum)) {
    return Eigen::VectorXf();
  }
  return weights;
}

LifecycleConfigSignature BuildLifecycleConfigSignature(
    const ResolvedRuntimeSignalPipelineConfig& resolved_config) {
  LifecycleConfigSignature signature;
  signature.enable_imm_lifecycle = resolved_config.public_config.lifecycle.enable_imm_lifecycle;
  signature.kalman_update_backend = resolved_config.public_config.tracking.kalman_update_backend;
  signature.imm_model_count =
      resolved_config.internal_config.lifecycle.imm_model_noise_diff_coeffs.size();
  signature.track_pool_thread_safety_mode =
      resolved_config.internal_config.lifecycle.track_pool_thread_safety_mode;
  return signature;
}

bool ShouldRebuildLifecycleAssembly(const LifecycleConfigSignature& before,
                                    const LifecycleConfigSignature& after) {
  return before.enable_imm_lifecycle != after.enable_imm_lifecycle ||
         before.kalman_update_backend != after.kalman_update_backend ||
         before.imm_model_count != after.imm_model_count ||
         before.track_pool_thread_safety_mode != after.track_pool_thread_safety_mode;
}

void AssignLifecycleSignature(const LifecycleConfigSignature& source,
                              LifecycleConfigSignature* destination) {
  if (destination == nullptr) {
    return;
  }
  destination->enable_imm_lifecycle = source.enable_imm_lifecycle;
  destination->kalman_update_backend = source.kalman_update_backend;
  destination->imm_model_count = source.imm_model_count;
  destination->track_pool_thread_safety_mode = source.track_pool_thread_safety_mode;
}

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
  return SignalComponentFactory::BuildLifecycleAssemblyArtifacts(fallback_config.public_config,
                                                                 fallback_config.internal_config);
}

class AutoConfiguredLifecycleManager final : public tracking::ITrackLifecycleManager {
 public:
  explicit AutoConfiguredLifecycleManager(
      const ResolvedRuntimeSignalPipelineConfig& resolved_config)
      : resolved_config_(resolved_config),
        signature_(BuildLifecycleConfigSignature(resolved_config_)),
        assembly_(BuildGuaranteedLifecycleAssemblyArtifacts(resolved_config_)) {}

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

  common::model::DecisionInputFrame BuildDecisionFrame(
      std::uint32_t cycle_index, std::uint64_t batch_id,
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

  void SyncRuntimeConfig(const ResolvedRuntimeSignalPipelineConfig& resolved_config) {
    const LifecycleConfigSignature incoming_signature =
        BuildLifecycleConfigSignature(resolved_config);
    if (ShouldRebuildLifecycleAssembly(signature_, incoming_signature)) {
      PROJECT_LOG_WARN(
          "[SignalPipeline] lifecycle config topology changed during runtime; rebuilding "
          "auto-lifecycle assembly and resetting lifecycle state.");
      resolved_config_ = resolved_config;
      AssignLifecycleSignature(incoming_signature, &signature_);
      assembly_ = BuildGuaranteedLifecycleAssemblyArtifacts(resolved_config_);
      return;
    }

    resolved_config_ = resolved_config;
    AssignLifecycleSignature(incoming_signature, &signature_);
    const tracking::LifecycleConfig lifecycle_config =
        SignalComponentFactory::BuildLifecycleConfig(resolved_config_.public_config,
                                                     resolved_config_.internal_config);
    const float kalman_noise_diff_coeff =
        resolved_config_.internal_config.tracking.kalman_noise_diff_coeff;
    const float kalman_measurement_noise_std =
        resolved_config_.public_config.tracking.kalman_measurement_noise_std;
    const std::vector<float>& imm_model_noise_diff_coeffs =
        resolved_config_.internal_config.lifecycle.imm_model_noise_diff_coeffs;
    const Eigen::MatrixXf imm_transition_probability = BuildImmTransitionProbabilityOrDefault(
        resolved_config_.internal_config, imm_model_noise_diff_coeffs.size());
    const Eigen::VectorXf imm_initial_weights =
        BuildImmInitialWeightsOrDefault(resolved_config_.internal_config,
                                        imm_model_noise_diff_coeffs.size());
    SyncRuntimeTuning(lifecycle_config, kalman_noise_diff_coeff, kalman_measurement_noise_std,
                      imm_model_noise_diff_coeffs, imm_transition_probability, imm_initial_weights);
  }

 private:
  ResolvedRuntimeSignalPipelineConfig resolved_config_;
  LifecycleConfigSignature signature_;
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
  return std::unique_ptr<tracking::ITrackLifecycleManager>(
      new AutoConfiguredLifecycleManager(resolved_config));
}

void RebuildOwnedComponentsForPipeline(
    const SignalPipelineConfig& base_config,
    const pipeline::internal::InternalSignalPipelineConfig& base_internal_config,
    const common::control::RadarControlProfile& control_profile, OwnedComponentSlots* slots) {
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

void SyncAutoLifecycleManagerForRuntimeConfig(
    const SignalPipelineConfig& runtime_config,
    const pipeline::internal::InternalSignalPipelineConfig& internal_runtime_config,
    tracking::ITrackLifecycleManager* auto_lifecycle_manager) {
  AutoConfiguredLifecycleManager* auto_configured_manager =
      dynamic_cast<AutoConfiguredLifecycleManager*>(auto_lifecycle_manager);
  if (auto_configured_manager == nullptr) {
    return;
  }
  ResolvedRuntimeSignalPipelineConfig resolved;
  resolved.public_config = runtime_config;
  resolved.internal_config = internal_runtime_config;
  auto_configured_manager->SyncRuntimeConfig(resolved);
}

}  // namespace internal
}  // namespace runtime
}  // namespace signal
}  // namespace airborne_radar
