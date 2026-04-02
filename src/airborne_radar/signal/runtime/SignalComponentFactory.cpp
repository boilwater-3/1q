/**
 * @file SignalComponentFactory.cpp
 * @brief SignalPipeline 私有组件工厂实现。
 */

#include "airborne_radar/signal/runtime/SignalComponentFactory.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace airborne_radar {
namespace signal {
namespace runtime {
namespace internal {

tracking::LifecycleConfig SignalComponentFactory::BuildLifecycleConfig(
    const SignalPipelineConfig& config,
    const pipeline::internal::InternalSignalPipelineConfig& internal_config) {
  tracking::LifecycleConfig lifecycle_config;
  lifecycle_config.confirm_hits = config.lifecycle.lifecycle_config.confirm_hits;
  lifecycle_config.max_miss_before_lost = config.lifecycle.lifecycle_config.max_miss_before_lost;
  lifecycle_config.max_lost_cycles = config.lifecycle.lifecycle_config.max_lost_cycles;
  lifecycle_config.imm_activation_policy = internal_config.lifecycle.imm_activation_policy;
  lifecycle_config.track_pool_thread_safety_mode =
      internal_config.lifecycle.track_pool_thread_safety_mode;
  return lifecycle_config;
}

tracking::TrackFilterConfig SignalComponentFactory::BuildTrackFilterConfig(
    const pipeline::internal::InternalSignalPipelineConfig& internal_config) {
  tracking::TrackFilterConfig filter_config;
  filter_config.speed_decay_ratio_on_loss = internal_config.tracking.speed_decay_ratio_on_loss;
  filter_config.rcs_decay_ratio_on_loss = internal_config.tracking.rcs_decay_ratio_on_loss;
  return filter_config;
}

association::DataAssociationConfig SignalComponentFactory::BuildAssociationConfig(
    const SignalPipelineConfig& config,
    const pipeline::internal::InternalSignalPipelineConfig& internal_config) {
  association::DataAssociationConfig association_config;
  association_config.unassigned_cost = internal_config.association.unassigned_cost;
  association_config.kalman_noise_diff_coeff = internal_config.tracking.kalman_noise_diff_coeff;
  association_config.kalman_measurement_noise_std = config.tracking.kalman_measurement_noise_std;
  return association_config;
}

OwnedSignalComponents SignalComponentFactory::BuildOwnedPipelineComponents(
    const SignalPipelineConfig& config,
    const pipeline::internal::InternalSignalPipelineConfig& internal_config) {
  OwnedSignalComponents components;
  components.association_config = BuildAssociationConfig(config, internal_config);
  components.track_filter_config = BuildTrackFilterConfig(internal_config);

  if (config.tracking.enable_kalman_filter) {
    components.kalman_predictor =
        CreateKalmanPredictor(internal_config.tracking.kalman_noise_diff_coeff);
    components.kalman_updater = CreateKalmanUpdater(config.tracking.kalman_measurement_noise_std,
                                                    config.tracking.kalman_update_backend);
  }

  if (config.detection.enable_physics_detection) {
    components.signal_detector.reset(new detection::SignalDetector(config.detection.radar_system));
  }
  return components;
}

LifecycleAssemblyArtifacts SignalComponentFactory::BuildLifecycleAssemblyArtifacts(
    const SignalPipelineConfig& config,
    const pipeline::internal::InternalSignalPipelineConfig& internal_config) {
  LifecycleAssemblyArtifacts artifacts;
  const tracking::LifecycleConfig lifecycle_config = BuildLifecycleConfig(config, internal_config);
  artifacts.pool.reset(
      new tracking::BoostTrackPool(internal_config.lifecycle.lifecycle_track_pool_initial_chunk,
                                   internal_config.lifecycle.lifecycle_track_pool_max_chunks));

  tracking::ITrackPool* effective_pool = artifacts.pool.get();
  if (lifecycle_config.track_pool_thread_safety_mode ==
      tracking::TrackPoolThreadSafetyMode::kMultiThreadGlobalLock) {
    artifacts.pool_wrapper.reset(new tracking::SynchronizedTrackPool(*artifacts.pool));
    effective_pool = artifacts.pool_wrapper.get();
  }

  if (config.lifecycle.enable_imm_lifecycle) {
    const std::size_t model_count = internal_config.lifecycle.imm_model_noise_diff_coeffs.size();
    bool imm_ready = true;
    if (model_count == 0U) {
      LogLifecycleAssemblyConfigViolation("IMM enabled but imm_model_noise_diff_coeffs is empty",
                                          model_count);
      imm_ready = false;
    }
    if (model_count > 3U) {
      PROJECT_LOG_WARN(
          "[SignalPipeline] IMM model count {} may increase lifecycle "
          "latency; 2-3 models are recommended for routine workloads",
          model_count);
    }

    if (imm_ready) {
      artifacts.imm_predictors_owned.reserve(model_count);
      artifacts.imm_updaters_owned.reserve(model_count);
      artifacts.imm_predictors.reserve(model_count);
      artifacts.imm_updaters.reserve(model_count);
      for (std::size_t i = 0; i < model_count; ++i) {
        const float noise_diff_coeff = internal_config.lifecycle.imm_model_noise_diff_coeffs[i];
        if (noise_diff_coeff <= 0.0f) {
          LogLifecycleAssemblyConfigViolation("IMM model noise_diff_coeff must be > 0",
                                              noise_diff_coeff);
          imm_ready = false;
          break;
        }

        artifacts.imm_predictors_owned.push_back(CreateKalmanPredictor(noise_diff_coeff));
        artifacts.imm_updaters_owned.push_back(CreateKalmanUpdater(
            config.tracking.kalman_measurement_noise_std, config.tracking.kalman_update_backend));
        artifacts.imm_predictors.push_back(artifacts.imm_predictors_owned.back().get());
        artifacts.imm_updaters.push_back(artifacts.imm_updaters_owned.back().get());
      }
    }

    if (imm_ready) {
      const Eigen::MatrixXf transition_probability =
          BuildImmTransitionProbability(internal_config, model_count);
      const Eigen::VectorXf initial_weights = BuildImmInitialWeights(internal_config, model_count);
      if (transition_probability.rows() == static_cast<Eigen::Index>(model_count) &&
          transition_probability.cols() == static_cast<Eigen::Index>(model_count) &&
          initial_weights.size() == static_cast<Eigen::Index>(model_count)) {
        artifacts.lifecycle_manager.reset(new tracking::TrackLifecycleManager(
            *effective_pool, lifecycle_config, artifacts.imm_predictors, artifacts.imm_updaters,
            transition_probability, initial_weights));
        return artifacts;
      }
      PROJECT_LOG_ERROR(
          "[SignalPipeline] IMM lifecycle matrix/weights validation failed; fallback to "
          "non-IMM lifecycle manager (model_count={})",
          model_count);
    }

    PROJECT_LOG_WARN(
        "[SignalPipeline] IMM lifecycle assembly failed; fallback to non-IMM lifecycle manager.");
  }

  if (config.tracking.enable_kalman_filter) {
    artifacts.kalman_predictor =
        CreateKalmanPredictor(internal_config.tracking.kalman_noise_diff_coeff);
    artifacts.kalman_updater = CreateKalmanUpdater(config.tracking.kalman_measurement_noise_std,
                                                   config.tracking.kalman_update_backend);
    artifacts.lifecycle_manager.reset(new tracking::TrackLifecycleManager(
        *effective_pool, lifecycle_config, artifacts.kalman_predictor.get(),
        artifacts.kalman_updater.get()));
    return artifacts;
  }

  artifacts.lifecycle_manager.reset(
      new tracking::TrackLifecycleManager(*effective_pool, lifecycle_config));
  return artifacts;
}

void SignalComponentFactory::LogLifecycleAssemblyConfigViolation(const char* message, float value) {
  PROJECT_LOG_ERROR("[SignalPipeline] lifecycle auto-assembly config violation: {} (value={})",
                    message, value);
}

void SignalComponentFactory::LogLifecycleAssemblyConfigViolation(const char* message,
                                                                 std::size_t value) {
  PROJECT_LOG_ERROR("[SignalPipeline] lifecycle auto-assembly config violation: {} (value={})",
                    message, value);
}

std::unique_ptr<tracking::KalmanPredictor> SignalComponentFactory::CreateKalmanPredictor(
    float noise_diff_coeff) {
  tracking::KalmanPredictorConfig predictor_config;
  predictor_config.noise_diff_coeff = std::max(noise_diff_coeff, 0.001f);
  return std::unique_ptr<tracking::KalmanPredictor>(
      new tracking::KalmanPredictor(predictor_config));
}

std::unique_ptr<tracking::IKalmanUpdater> SignalComponentFactory::CreateKalmanUpdater(
    float measurement_noise_std, common::config::KalmanUpdateBackend backend) {
  tracking::KalmanUpdaterConfig updater_config;
  updater_config.measurement_noise_std = std::max(measurement_noise_std, 0.001f);
  if (backend == common::config::KalmanUpdateBackend::kUdKf) {
    return std::unique_ptr<tracking::IKalmanUpdater>(new tracking::UdkfUpdater(updater_config));
  }
  if (backend == common::config::KalmanUpdateBackend::kSrif) {
    return std::unique_ptr<tracking::IKalmanUpdater>(new tracking::SrifUpdater(updater_config));
  }
  return std::unique_ptr<tracking::IKalmanUpdater>(new tracking::KalmanUpdater(updater_config));
}

Eigen::MatrixXf SignalComponentFactory::BuildImmTransitionProbability(
    const pipeline::internal::InternalSignalPipelineConfig& internal_config,
    std::size_t model_count) {
  if (internal_config.lifecycle.imm_transition_probability.empty()) {
    Eigen::MatrixXf matrix = Eigen::MatrixXf::Constant(
        static_cast<Eigen::Index>(model_count), static_cast<Eigen::Index>(model_count),
        model_count > 1U ? 0.05f / static_cast<float>(model_count - 1U) : 1.0f);
    matrix.diagonal().setConstant(model_count > 1U ? 0.95f : 1.0f);
    return matrix;
  }

  if (internal_config.lifecycle.imm_transition_probability.size() != model_count * model_count) {
    LogLifecycleAssemblyConfigViolation(
        "imm_transition_probability size must equal model_count*model_count",
        internal_config.lifecycle.imm_transition_probability.size());
    return Eigen::MatrixXf();
  }

  Eigen::MatrixXf matrix(static_cast<Eigen::Index>(model_count),
                         static_cast<Eigen::Index>(model_count));
  for (std::size_t r = 0; r < model_count; ++r) {
    float row_sum = 0.0f;
    for (std::size_t c = 0; c < model_count; ++c) {
      const float value = internal_config.lifecycle.imm_transition_probability[r * model_count + c];
      if (value < 0.0f || value > 1.0f) {
        LogLifecycleAssemblyConfigViolation("IMM transition probability must be in [0,1]", value);
        return Eigen::MatrixXf();
      }
      matrix(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) = value;
      row_sum += value;
    }
    if (std::fabs(row_sum - 1.0f) > 1e-3f) {
      LogLifecycleAssemblyConfigViolation("each IMM transition matrix row must sum to 1", row_sum);
      return Eigen::MatrixXf();
    }
  }
  return matrix;
}

Eigen::VectorXf SignalComponentFactory::BuildImmInitialWeights(
    const pipeline::internal::InternalSignalPipelineConfig& internal_config,
    std::size_t model_count) {
  if (internal_config.lifecycle.imm_initial_weights.empty()) {
    return Eigen::VectorXf::Constant(static_cast<Eigen::Index>(model_count),
                                     1.0f / static_cast<float>(model_count));
  }

  if (internal_config.lifecycle.imm_initial_weights.size() != model_count) {
    LogLifecycleAssemblyConfigViolation("imm_initial_weights size must equal model_count",
                                        internal_config.lifecycle.imm_initial_weights.size());
    return Eigen::VectorXf();
  }

  Eigen::VectorXf weights(static_cast<Eigen::Index>(model_count));
  float sum = 0.0f;
  for (std::size_t i = 0; i < model_count; ++i) {
    const float value = internal_config.lifecycle.imm_initial_weights[i];
    if (value < 0.0f || value > 1.0f) {
      LogLifecycleAssemblyConfigViolation("IMM initial weight must be in [0,1]", value);
      return Eigen::VectorXf();
    }
    weights(static_cast<Eigen::Index>(i)) = value;
    sum += value;
  }

  if (std::fabs(sum - 1.0f) > 1e-3f) {
    LogLifecycleAssemblyConfigViolation("IMM initial weights must sum to 1", sum);
    return Eigen::VectorXf();
  }
  return weights;
}

}  // namespace internal
}  // namespace runtime
}  // namespace signal
}  // namespace airborne_radar
