/**
 * @file SignalComponentFactory.cpp
 * @brief SignalPipeline 私有组件工厂实现。
 */

#include "airborne_radar/signal/pipeline/SignalComponentFactory.h"

#include <Eigen/Core>
#include <algorithm>
#include <memory>
#include <vector>

#include "airborne_radar/signal/pipeline/ImmMatrixDefaults.h"

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
    tracking::KalmanPredictorConfig pred_cfg;
    pred_cfg.noise_diff_coeff = std::max(config.tracking.kalman_noise_diff_coeff, 0.001f);
    components.kalman_predictor.reset(new tracking::KalmanPredictor(pred_cfg));
    tracking::KalmanUpdaterConfig upd_cfg;
    upd_cfg.measurement_noise_std =
        std::max(config.tracking.engineering.kalman_measurement_noise_std, 0.001f);
    components.kalman_updater.reset(new tracking::KalmanUpdater(upd_cfg));
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

        tracking::KalmanPredictorConfig imm_pred_cfg;
        imm_pred_cfg.noise_diff_coeff = std::max(noise_diff_coeff, 0.001f);
        artifacts.imm_predictors_owned.push_back(
            std::unique_ptr<tracking::IKalmanPredictor>(
                new tracking::KalmanPredictor(imm_pred_cfg)));
        tracking::KalmanUpdaterConfig imm_upd_cfg;
        imm_upd_cfg.measurement_noise_std =
            std::max(config.tracking.engineering.kalman_measurement_noise_std, 0.001f);
        artifacts.imm_updaters_owned.push_back(
            std::unique_ptr<tracking::IKalmanUpdater>(
                new tracking::KalmanUpdater(imm_upd_cfg)));
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
    tracking::KalmanPredictorConfig pred_cfg2;
    pred_cfg2.noise_diff_coeff = std::max(config.tracking.kalman_noise_diff_coeff, 0.001f);
    artifacts.kalman_predictor.reset(new tracking::KalmanPredictor(pred_cfg2));
    tracking::KalmanUpdaterConfig upd_cfg2;
    upd_cfg2.measurement_noise_std =
        std::max(config.tracking.engineering.kalman_measurement_noise_std, 0.001f);
    artifacts.kalman_updater.reset(new tracking::KalmanUpdater(upd_cfg2));
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

Eigen::MatrixXf SignalComponentFactory::BuildImmTransitionProbability(const ExecutionConfig& config,
                                                                      std::size_t model_count) {
  const imm_defaults::ViolationReporter report = [](const char* message, float value) {
    LogLifecycleAssemblyConfigViolation(message, value);
  };
  return imm_defaults::BuildTransitionProbability(config, model_count, report);
}

Eigen::VectorXf SignalComponentFactory::BuildImmInitialWeights(const ExecutionConfig& config,
                                                               std::size_t model_count) {
  const imm_defaults::ViolationReporter report = [](const char* message, float value) {
    LogLifecycleAssemblyConfigViolation(message, value);
  };
  return imm_defaults::BuildInitialWeights(config, model_count, report);
}



}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
