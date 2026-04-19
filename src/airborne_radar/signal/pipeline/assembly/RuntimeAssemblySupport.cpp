#include "airborne_radar/signal/pipeline/assembly/RuntimeAssemblySupport.h"

#include <cmath>
#include <utility>

#include "airborne_radar/config/engineering/SignalEngineeringConfig.h"
#include "airborne_radar/signal/pipeline/assembly/SignalComponentFactory.h"
#include "airborne_radar/signal/pipeline/effects/ControlProfileEffects.h"
#include "airborne_radar/signal/tracking/TrackLifecycleManager.h"
#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace assembly {
namespace internal {

namespace {

struct LifecycleConfigSignature {
  bool enable_imm_lifecycle{false};
  config::engineering::KalmanUpdateBackend kalman_update_backend{
      config::engineering::KalmanUpdateBackend::kStandardKfJoseph};
  std::size_t imm_model_count{0U};
  tracking::TrackPoolThreadSafetyMode track_pool_thread_safety_mode{
      tracking::TrackPoolThreadSafetyMode::kSingleThreadNoLock};
};

bool IsNearOne(float value) { return std::fabs(value - 1.0f) <= 1.0e-3f; }

Eigen::MatrixXf BuildImmTransitionProbabilityOrDefault(const ExecutionConfig& config,
                                                       std::size_t model_count) {
  if (model_count == 0U) {
    return Eigen::MatrixXf();
  }
  if (config.imm_transition_probability.empty()) {
    Eigen::MatrixXf matrix = Eigen::MatrixXf::Constant(
        static_cast<Eigen::Index>(model_count), static_cast<Eigen::Index>(model_count),
        model_count > 1U ? 0.05f / static_cast<float>(model_count - 1U) : 1.0f);
    matrix.diagonal().setConstant(model_count > 1U ? 0.95f : 1.0f);
    return matrix;
  }
  if (config.imm_transition_probability.size() != model_count * model_count) {
    return Eigen::MatrixXf();
  }
  Eigen::MatrixXf matrix(static_cast<Eigen::Index>(model_count),
                         static_cast<Eigen::Index>(model_count));
  for (std::size_t r = 0; r < model_count; ++r) {
    float row_sum = 0.0f;
    for (std::size_t c = 0; c < model_count; ++c) {
      const float value = config.imm_transition_probability[r * model_count + c];
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

Eigen::VectorXf BuildImmInitialWeightsOrDefault(const ExecutionConfig& config,
                                                std::size_t model_count) {
  if (model_count == 0U) {
    return Eigen::VectorXf();
  }
  if (config.imm_initial_weights.empty()) {
    return Eigen::VectorXf::Constant(static_cast<Eigen::Index>(model_count),
                                     1.0f / static_cast<float>(model_count));
  }
  if (config.imm_initial_weights.size() != model_count) {
    return Eigen::VectorXf();
  }
  Eigen::VectorXf weights(static_cast<Eigen::Index>(model_count));
  float sum = 0.0f;
  for (std::size_t i = 0; i < model_count; ++i) {
    const float value = config.imm_initial_weights[i];
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

LifecycleConfigSignature BuildLifecycleConfigSignature(const ExecutionConfig& config) {
  LifecycleConfigSignature signature;
  signature.enable_imm_lifecycle = config.lifecycle_engineering.enable_imm_lifecycle;
  signature.kalman_update_backend = config.tracking_engineering.kalman_update_backend;
  signature.imm_model_count = config.imm_model_noise_diff_coeffs.size();
  signature.track_pool_thread_safety_mode = config.track_pool_thread_safety_mode;
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

LifecycleAssemblyArtifacts BuildLifecycleAssemblyArtifactsOrLogFailure(
    const ExecutionConfig& config) {
  LifecycleAssemblyArtifacts artifacts =
      SignalComponentFactory::BuildLifecycleAssemblyArtifacts(config);
  if (artifacts.lifecycle_manager == nullptr) {
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

  model::TargetFeatureList BuildFeatureSnapshot() const override {
    if (assembly_.lifecycle_manager == nullptr) {
      return model::TargetFeatureList();
    }
    return assembly_.lifecycle_manager->BuildFeatureSnapshot();
  }

  model::DecisionTrackSnapshotList BuildDecisionSnapshot() const override {
    if (assembly_.lifecycle_manager == nullptr) {
      return model::DecisionTrackSnapshotList();
    }
    return assembly_.lifecycle_manager->BuildDecisionSnapshot();
  }

  model::DecisionInputFrame BuildDecisionFrame(std::uint32_t cycle_index, std::uint64_t batch_id,
                                               bool environment_jamming_detected) const override {
    if (assembly_.lifecycle_manager == nullptr) {
      model::DecisionInputFrame frame;
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

  bool SyncRuntimeConfig(const ExecutionConfig& config) {
    const LifecycleConfigSignature incoming_signature = BuildLifecycleConfigSignature(config);
    if (ShouldRebuildLifecycleAssembly(signature_, incoming_signature)) {
      PROJECT_LOG_WARN(
          "[SignalPipeline] lifecycle config topology changed during runtime; rebuilding "
          "auto-lifecycle assembly and resetting lifecycle state.");
      LifecycleAssemblyArtifacts rebuilt_assembly =
          BuildLifecycleAssemblyArtifactsOrLogFailure(config);
      if (rebuilt_assembly.lifecycle_manager == nullptr) {
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
    const float kalman_noise_diff_coeff = config_.tracking_kalman_noise_diff_coeff;
    const float kalman_measurement_noise_std =
        config_.tracking_engineering.kalman_measurement_noise_std;
    const std::vector<float>& imm_model_noise_diff_coeffs = config_.imm_model_noise_diff_coeffs;
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
    const extension::control::RadarControlProfile& control_profile) {
  ResolvedRuntimePipelineConfig resolved;
  resolved.config = base_config;
  pipeline::internal::ApplyControlProfileToConfig(control_profile, &resolved.config);
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
    const extension::control::RadarControlProfile& control_profile, OwnedComponentSlots* slots) {
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

}  // namespace internal
}  // namespace assembly
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
