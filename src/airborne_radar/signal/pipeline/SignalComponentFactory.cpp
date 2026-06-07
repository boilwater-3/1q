/**
 * @file SignalComponentFactory.cpp
 * @brief SignalPipeline 私有组件工厂实现。
 */

#include "airborne_radar/signal/pipeline/SignalComponentFactory.h"

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

namespace airborne_radar {
namespace signal {
namespace pipeline {

tracking::LifecycleConfig SignalComponentFactory::BuildLifecycleConfig(
    const ExecutionConfig& config) {
  tracking::LifecycleConfig lifecycle_config;
  const ::airborne_radar::config::engineering::LifecycleConfig& lifecycle_runtime =
      config.lifecycle.engineering.lifecycle_config;
  lifecycle_config.confirm_hits = lifecycle_runtime.confirm_hits;
  lifecycle_config.max_miss_before_lost = lifecycle_runtime.max_miss_before_lost;
  lifecycle_config.max_lost_cycles = lifecycle_runtime.max_lost_cycles;
  lifecycle_config.imm_activation_policy = config.lifecycle.imm_activation_policy;
  lifecycle_config.track_pool_thread_safety_mode = config.lifecycle.track_pool_thread_safety_mode;
  return lifecycle_config;
}

tracking::TrackFilterConfig SignalComponentFactory::BuildTrackFilterConfig(
    const ExecutionConfig& config) {
  tracking::TrackFilterConfig filter_config;
  filter_config.speed_decay_ratio_on_loss = config.tracking.engineering.speed_decay_ratio_on_loss;
  filter_config.rcs_decay_ratio_on_loss = config.tracking.engineering.rcs_decay_ratio_on_loss;
  return filter_config;
}

association::DataAssociationConfig SignalComponentFactory::BuildAssociationConfig(
    const ExecutionConfig& config) {
  association::DataAssociationConfig association_config;
  association_config.unassigned_cost = config.association.policy.unassigned_cost;
  association_config.kalman_noise_diff_coeff = config.tracking.kalman_noise_diff_coeff;
  association_config.kalman_measurement_noise_std =
      config.tracking.engineering.kalman_measurement_noise_std;
  return association_config;
}

OwnedSignalComponents SignalComponentFactory::BuildOwnedPipelineComponents(
    const ExecutionConfig& config) {
  OwnedSignalComponents components;
  components.association_config = BuildAssociationConfig(config);
  components.track_filter_config = BuildTrackFilterConfig(config);

  if (config.tracking.engineering.enable_kalman_filter) {
    components.kalman_predictor = CreateKalmanPredictor(
        config.tracking.kalman_noise_diff_coeff, config.tracking.engineering.kalman_update_backend);
    components.kalman_updater =
        CreateKalmanUpdater(config.tracking.engineering.kalman_measurement_noise_std,
                            config.tracking.engineering.kalman_update_backend);
  }

  if (config.detection.engineering.enable_physics_detection) {
    components.signal_detector.reset(new detection::SignalDetector(config.detection.engineering));
  }
  return components;
}

LifecycleAssemblyArtifacts SignalComponentFactory::BuildLifecycleAssemblyArtifacts(
    const ExecutionConfig& config) {
  LifecycleAssemblyArtifacts artifacts;
  const tracking::LifecycleConfig lifecycle_config = BuildLifecycleConfig(config);
  artifacts.pool.reset(new tracking::BoostTrackPool(config.lifecycle.track_pool_initial_chunk,
                                                    config.lifecycle.track_pool_max_chunks));

  tracking::ITrackPool* effective_pool = artifacts.pool.get();
  if (lifecycle_config.track_pool_thread_safety_mode ==
      tracking::TrackPoolThreadSafetyMode::kMultiThreadGlobalLock) {
    artifacts.pool_wrapper.reset(new tracking::SynchronizedTrackPool(*artifacts.pool));
    effective_pool = artifacts.pool_wrapper.get();
  }

  if (config.lifecycle.engineering.enable_imm_lifecycle) {
    const std::size_t model_count = config.lifecycle.imm_model_noise_diff_coeffs.size();
    bool imm_ready = true;
    if (model_count == 0U) {
      LogLifecycleAssemblyConfigViolation("IMM enabled but lifecycle.imm_model_noise_diff_coeffs is empty",
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
        const float noise_diff_coeff = config.lifecycle.imm_model_noise_diff_coeffs[i];
        if (noise_diff_coeff <= 0.0f) {
          LogLifecycleAssemblyConfigViolation("IMM model noise_diff_coeff must be > 0",
                                              noise_diff_coeff);
          imm_ready = false;
          break;
        }

        artifacts.imm_predictors_owned.push_back(CreateKalmanPredictor(
            noise_diff_coeff, config.tracking.engineering.kalman_update_backend));
        artifacts.imm_updaters_owned.push_back(
            CreateKalmanUpdater(config.tracking.engineering.kalman_measurement_noise_std,
                                config.tracking.engineering.kalman_update_backend));
        artifacts.imm_predictors.push_back(artifacts.imm_predictors_owned.back().get());
        artifacts.imm_updaters.push_back(artifacts.imm_updaters_owned.back().get());
      }
    }

    if (imm_ready) {
      const Eigen::MatrixXf transition_probability =
          BuildImmTransitionProbability(config, model_count);
      const Eigen::VectorXf initial_weights = BuildImmInitialWeights(config, model_count);
      if (transition_probability.rows() == static_cast<Eigen::Index>(model_count) &&
          transition_probability.cols() == static_cast<Eigen::Index>(model_count) &&
          initial_weights.size() == static_cast<Eigen::Index>(model_count)) {
        artifacts.lifecycle_manager.reset(new tracking::TrackLifecycleManager(
            *effective_pool, lifecycle_config, artifacts.imm_predictors, artifacts.imm_updaters,
            transition_probability, initial_weights));
        return artifacts;
      }
      PROJECT_LOG_ERROR(
          "[SignalPipeline] IMM lifecycle matrix/weights validation failed; lifecycle "
          "assembly aborted (model_count={})",
          model_count);
    }

    PROJECT_LOG_ERROR(
        "[SignalPipeline] IMM lifecycle assembly failed; no fallback manager created.");
    return artifacts;
  }

  if (config.tracking.engineering.enable_kalman_filter) {
    artifacts.kalman_predictor = CreateKalmanPredictor(
        config.tracking.kalman_noise_diff_coeff, config.tracking.engineering.kalman_update_backend);
    artifacts.kalman_updater =
        CreateKalmanUpdater(config.tracking.engineering.kalman_measurement_noise_std,
                            config.tracking.engineering.kalman_update_backend);
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

std::unique_ptr<tracking::IKalmanPredictor> SignalComponentFactory::CreateKalmanPredictor(
    float noise_diff_coeff, config::engineering::KalmanUpdateBackend backend) {
  tracking::KalmanPredictorConfig predictor_config;
  predictor_config.noise_diff_coeff = std::max(noise_diff_coeff, 0.001f);
  if (backend == config::engineering::KalmanUpdateBackend::kUdKf) {
    return std::unique_ptr<tracking::IKalmanPredictor>(
        new tracking::UdkfPredictor(predictor_config));
  }
  if (backend == config::engineering::KalmanUpdateBackend::kSrif) {
    return std::unique_ptr<tracking::IKalmanPredictor>(
        new tracking::SrifPredictor(predictor_config));
  }
  if (backend == config::engineering::KalmanUpdateBackend::kEkf) {
    static const tracking::LinearCvTransitionModel ekf_transition;
    tracking::EkfPredictorConfig ekf_config;
    ekf_config.noise_diff_coeff = std::max(noise_diff_coeff, 0.001f);
    return std::unique_ptr<tracking::IKalmanPredictor>(
        new tracking::EkfPredictor(&ekf_transition, ekf_config));
  }
  return std::unique_ptr<tracking::IKalmanPredictor>(
      new tracking::KalmanPredictor(predictor_config));
}

std::unique_ptr<tracking::IKalmanUpdater> SignalComponentFactory::CreateKalmanUpdater(
    float measurement_noise_std, config::engineering::KalmanUpdateBackend backend) {
  tracking::KalmanUpdaterConfig updater_config;
  updater_config.measurement_noise_std = std::max(measurement_noise_std, 0.001f);
  if (backend == config::engineering::KalmanUpdateBackend::kUdKf) {
    return std::unique_ptr<tracking::IKalmanUpdater>(new tracking::UdkfUpdater(updater_config));
  }
  if (backend == config::engineering::KalmanUpdateBackend::kSrif) {
    return std::unique_ptr<tracking::IKalmanUpdater>(new tracking::SrifUpdater(updater_config));
  }
  if (backend == config::engineering::KalmanUpdateBackend::kEkf) {
    static const tracking::LinearPositionMeasurementModel ekf_measurement;
    tracking::EkfUpdaterConfig ekf_updater_config;
    ekf_updater_config.measurement_noise_std = std::max(measurement_noise_std, 0.001f);
    return std::unique_ptr<tracking::IKalmanUpdater>(
        new tracking::EkfUpdater(&ekf_measurement, ekf_updater_config));
  }
  return std::unique_ptr<tracking::IKalmanUpdater>(new tracking::KalmanUpdater(updater_config));
}

Eigen::MatrixXf SignalComponentFactory::BuildImmTransitionProbability(const ExecutionConfig& config,
                                                                      std::size_t model_count) {
  if (config.lifecycle.imm_transition_probability.empty()) {
    Eigen::MatrixXf matrix = Eigen::MatrixXf::Constant(
        static_cast<Eigen::Index>(model_count), static_cast<Eigen::Index>(model_count),
        model_count > 1U ? 0.05f / static_cast<float>(model_count - 1U) : 1.0f);
    matrix.diagonal().setConstant(model_count > 1U ? 0.95f : 1.0f);
    return matrix;
  }

  if (config.lifecycle.imm_transition_probability.size() != model_count * model_count) {
    LogLifecycleAssemblyConfigViolation(
        "lifecycle.imm_transition_probability size must equal model_count*model_count",
        config.lifecycle.imm_transition_probability.size());
    return Eigen::MatrixXf();
  }

  Eigen::MatrixXf matrix(static_cast<Eigen::Index>(model_count),
                         static_cast<Eigen::Index>(model_count));
  for (std::size_t r = 0; r < model_count; ++r) {
    float row_sum = 0.0f;
    for (std::size_t c = 0; c < model_count; ++c) {
      const float value = config.lifecycle.imm_transition_probability[r * model_count + c];
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

Eigen::VectorXf SignalComponentFactory::BuildImmInitialWeights(const ExecutionConfig& config,
                                                               std::size_t model_count) {
  if (config.lifecycle.imm_initial_weights.empty()) {
    return Eigen::VectorXf::Constant(static_cast<Eigen::Index>(model_count),
                                     1.0f / static_cast<float>(model_count));
  }

  if (config.lifecycle.imm_initial_weights.size() != model_count) {
    LogLifecycleAssemblyConfigViolation("lifecycle.imm_initial_weights size must equal model_count",
                                        config.lifecycle.imm_initial_weights.size());
    return Eigen::VectorXf();
  }

  Eigen::VectorXf weights(static_cast<Eigen::Index>(model_count));
  float sum = 0.0f;
  for (std::size_t i = 0; i < model_count; ++i) {
    const float value = config.lifecycle.imm_initial_weights[i];
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



}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
