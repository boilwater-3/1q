// Copyright 2026. All Rights Reserved.
//
// Description: 定义 SignalPipeline 私有组件工厂，负责配置映射与组件装配。

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_COMPONENT_FACTORY_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_COMPONENT_FACTORY_H_

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <spdlog/spdlog.h>

#include "1q/airborne_radar/signal/pipeline/SignalPipeline.h"
#include "1q/airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "1q/airborne_radar/signal/tracking/TrackLifecycleManager.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/tracking/BoostTrackPool.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

/// @brief Pipeline 自持有组件集合。
struct OwnedSignalComponents {
  /// @brief 关联引擎配置。
  association::DataAssociationConfig association_config{};

  /// @brief 轨迹滤波配置。
  tracking::TrackFilterConfig track_filter_config{};

  /// @brief 可选 Kalman 预测器。
  std::unique_ptr<tracking::KalmanPredictor> kalman_predictor;

  /// @brief 可选 Kalman 更新器。
  std::unique_ptr<tracking::KalmanUpdater> kalman_updater;

  /// @brief 可选物理探测器。
  std::unique_ptr<detection::SignalDetector> signal_detector;
};

/// @brief Lifecycle 自动装配后的组件集合。
struct LifecycleAssemblyArtifacts {
  /// @brief 生命周期对象池。
  std::unique_ptr<tracking::BoostTrackPool> pool;

  /// @brief 单模型 Kalman 预测器。
  std::unique_ptr<tracking::KalmanPredictor> kalman_predictor;

  /// @brief 单模型 Kalman 更新器。
  std::unique_ptr<tracking::KalmanUpdater> kalman_updater;

  /// @brief IMM 自持有预测器集合。
  std::vector<std::unique_ptr<tracking::KalmanPredictor> > imm_predictors_owned;

  /// @brief IMM 自持有更新器集合。
  std::vector<std::unique_ptr<tracking::KalmanUpdater> > imm_updaters_owned;

  /// @brief 传递给生命周期管理器的 IMM 预测器视图。
  std::vector<const tracking::IKalmanPredictor*> imm_predictors;

  /// @brief 传递给生命周期管理器的 IMM 更新器视图。
  std::vector<const tracking::IKalmanUpdater*> imm_updaters;

  /// @brief 自动装配后的生命周期管理器。
  std::unique_ptr<tracking::ITrackLifecycleManager> lifecycle_manager;
};

/// @brief SignalComponentFactory 统一负责 Signal 层组件装配。
class SignalComponentFactory final {
 public:
  /// @brief 从顶层配置构造轨迹滤波配置。
  /// @param config Signal 顶层配置。
  /// @return TrackFilter 使用的配置。
  static tracking::TrackFilterConfig BuildTrackFilterConfig(
      const SignalPipelineConfig& config) {
    tracking::TrackFilterConfig filter_config;
    filter_config.speed_decay_ratio_on_loss =
        config.tracking.speed_decay_ratio_on_loss;
    filter_config.rcs_decay_ratio_on_loss =
        config.tracking.rcs_decay_ratio_on_loss;
    filter_config.jamming_acceleration_penalty =
        config.tracking.jamming_acceleration_penalty;
    filter_config.stable_acceleration_gain =
        config.tracking.stable_acceleration_gain;
    return filter_config;
  }

  /// @brief 从顶层配置构造数据关联配置。
  /// @param config Signal 顶层配置。
  /// @return DataAssociationEngine 使用的配置。
  static association::DataAssociationConfig BuildAssociationConfig(
      const SignalPipelineConfig& config) {
    association::DataAssociationConfig association_config;
    association_config.kalman_noise_diff_coeff =
        config.tracking.kalman_noise_diff_coeff;
    association_config.kalman_measurement_noise_std =
        config.tracking.kalman_measurement_noise_std;
    return association_config;
  }

  /// @brief 构造 Pipeline 自持有组件。
  /// @param config Signal 顶层配置。
  /// @return 组件与其配置集合。
  static OwnedSignalComponents BuildOwnedPipelineComponents(
      const SignalPipelineConfig& config) {
    OwnedSignalComponents components;
    components.association_config = BuildAssociationConfig(config);
    components.track_filter_config = BuildTrackFilterConfig(config);

    if (config.tracking.enable_kalman_filter) {
      components.kalman_predictor =
          CreateKalmanPredictor(config.tracking.kalman_noise_diff_coeff);
      components.kalman_updater = CreateKalmanUpdater(
          config.tracking.kalman_measurement_noise_std);
    }

    if (config.detection.enable_physics_detection) {
      components.signal_detector.reset(
          new detection::SignalDetector(config.detection.radar_system));
    }
    return components;
  }

  /// @brief 构造 Lifecycle 自动装配组件。
  /// @param config Signal 顶层配置。
  /// @return 生命周期装配结果。
  static LifecycleAssemblyArtifacts BuildLifecycleAssemblyArtifacts(
      const SignalPipelineConfig& config) {
    LifecycleAssemblyArtifacts artifacts;
    artifacts.pool.reset(new tracking::BoostTrackPool(
        config.lifecycle.lifecycle_track_pool_initial_chunk,
        config.lifecycle.lifecycle_track_pool_max_chunks));

    if (config.lifecycle.enable_imm_lifecycle) {
      const std::size_t model_count =
          config.lifecycle.imm_model_noise_diff_coeffs.size();
      if (model_count == 0U) {
        AbortLifecycleAssemblyConfigViolation(
            "IMM enabled but imm_model_noise_diff_coeffs is empty",
            model_count);
      }
      if (model_count > 3U) {
        spdlog::warn(
            "[SignalPipeline] IMM model count {} may increase lifecycle "
            "latency; 2-3 models are recommended for routine workloads",
            model_count);
      }

      artifacts.imm_predictors_owned.reserve(model_count);
      artifacts.imm_updaters_owned.reserve(model_count);
      artifacts.imm_predictors.reserve(model_count);
      artifacts.imm_updaters.reserve(model_count);
      for (std::size_t i = 0; i < model_count; ++i) {
        const float noise_diff_coeff =
            config.lifecycle.imm_model_noise_diff_coeffs[i];
        if (noise_diff_coeff <= 0.0f) {
          AbortLifecycleAssemblyConfigViolation(
              "IMM model noise_diff_coeff must be > 0", noise_diff_coeff);
        }

        artifacts.imm_predictors_owned.push_back(
            CreateKalmanPredictor(noise_diff_coeff));
        artifacts.imm_updaters_owned.push_back(CreateKalmanUpdater(
            config.tracking.kalman_measurement_noise_std));
        artifacts.imm_predictors.push_back(
            artifacts.imm_predictors_owned.back().get());
        artifacts.imm_updaters.push_back(
            artifacts.imm_updaters_owned.back().get());
      }

      artifacts.lifecycle_manager.reset(new tracking::TrackLifecycleManager(
          *artifacts.pool, config.lifecycle.lifecycle_config,
          artifacts.imm_predictors, artifacts.imm_updaters,
          BuildImmTransitionProbability(config, model_count),
          BuildImmInitialWeights(config, model_count)));
      return artifacts;
    }

    if (config.tracking.enable_kalman_filter) {
      artifacts.kalman_predictor =
          CreateKalmanPredictor(config.tracking.kalman_noise_diff_coeff);
      artifacts.kalman_updater = CreateKalmanUpdater(
          config.tracking.kalman_measurement_noise_std);
      artifacts.lifecycle_manager.reset(new tracking::TrackLifecycleManager(
          *artifacts.pool, config.lifecycle.lifecycle_config,
          artifacts.kalman_predictor.get(), artifacts.kalman_updater.get()));
      return artifacts;
    }

    artifacts.lifecycle_manager.reset(new tracking::TrackLifecycleManager(
        *artifacts.pool, config.lifecycle.lifecycle_config));
    return artifacts;
  }

 private:
  /// @brief 终止生命周期自动装配配置违规。
  /// @param message 错误消息。
  /// @param value 附带数值。
  static void AbortLifecycleAssemblyConfigViolation(const char* message,
                                                   float value) {
    spdlog::critical(
        "[SignalPipeline] lifecycle auto-assembly config violation: {} "
        "(value={})",
        message, value);
    std::abort();
  }

  /// @brief 终止生命周期自动装配配置违规。
  /// @param message 错误消息。
  /// @param value 附带数值。
  static void AbortLifecycleAssemblyConfigViolation(const char* message,
                                                   std::size_t value) {
    spdlog::critical(
        "[SignalPipeline] lifecycle auto-assembly config violation: {} "
        "(value={})",
        message, value);
    std::abort();
  }

  /// @brief 构造 Kalman 预测器。
  /// @param noise_diff_coeff 过程噪声扩散系数。
  /// @return 已创建的预测器。
  static std::unique_ptr<tracking::KalmanPredictor> CreateKalmanPredictor(
      float noise_diff_coeff) {
    tracking::KalmanPredictorConfig predictor_config;
    predictor_config.noise_diff_coeff = std::max(noise_diff_coeff, 0.001f);
    return std::unique_ptr<tracking::KalmanPredictor>(
        new tracking::KalmanPredictor(predictor_config));
  }

  /// @brief 构造 Kalman 更新器。
  /// @param measurement_noise_std 量测噪声标准差。
  /// @return 已创建的更新器。
  static std::unique_ptr<tracking::KalmanUpdater> CreateKalmanUpdater(
      float measurement_noise_std) {
    tracking::KalmanUpdaterConfig updater_config;
    updater_config.measurement_noise_std =
        std::max(measurement_noise_std, 0.001f);
    return std::unique_ptr<tracking::KalmanUpdater>(
        new tracking::KalmanUpdater(updater_config));
  }

  /// @brief 构建 IMM 转移矩阵。
  /// @param config 顶层配置。
  /// @param model_count IMM 模型数。
  /// @return 转移概率矩阵。
  static Eigen::MatrixXf BuildImmTransitionProbability(
      const SignalPipelineConfig& config,
      std::size_t model_count) {
    if (config.lifecycle.imm_transition_probability.empty()) {
      Eigen::MatrixXf matrix = Eigen::MatrixXf::Constant(
          static_cast<Eigen::Index>(model_count),
          static_cast<Eigen::Index>(model_count),
          model_count > 1U ? 0.05f / static_cast<float>(model_count - 1U)
                           : 1.0f);
      matrix.diagonal().setConstant(model_count > 1U ? 0.95f : 1.0f);
      return matrix;
    }

    if (config.lifecycle.imm_transition_probability.size() !=
        model_count * model_count) {
      AbortLifecycleAssemblyConfigViolation(
          "imm_transition_probability size must equal model_count*model_count",
          config.lifecycle.imm_transition_probability.size());
    }

    Eigen::MatrixXf matrix(static_cast<Eigen::Index>(model_count),
                           static_cast<Eigen::Index>(model_count));
    for (std::size_t r = 0; r < model_count; ++r) {
      float row_sum = 0.0f;
      for (std::size_t c = 0; c < model_count; ++c) {
        const float value = config.lifecycle
                                .imm_transition_probability[r * model_count +
                                                            c];
        if (value < 0.0f || value > 1.0f) {
          AbortLifecycleAssemblyConfigViolation(
              "IMM transition probability must be in [0,1]", value);
        }
        matrix(static_cast<Eigen::Index>(r), static_cast<Eigen::Index>(c)) =
            value;
        row_sum += value;
      }
      if (std::fabs(row_sum - 1.0f) > 1e-3f) {
        AbortLifecycleAssemblyConfigViolation(
            "each IMM transition matrix row must sum to 1", row_sum);
      }
    }
    return matrix;
  }

  /// @brief 构建 IMM 初始权重。
  /// @param config 顶层配置。
  /// @param model_count IMM 模型数。
  /// @return 初始权重向量。
  static Eigen::VectorXf BuildImmInitialWeights(
      const SignalPipelineConfig& config,
      std::size_t model_count) {
    if (config.lifecycle.imm_initial_weights.empty()) {
      return Eigen::VectorXf::Constant(
          static_cast<Eigen::Index>(model_count),
          1.0f / static_cast<float>(model_count));
    }

    if (config.lifecycle.imm_initial_weights.size() != model_count) {
      AbortLifecycleAssemblyConfigViolation(
          "imm_initial_weights size must equal model_count",
          config.lifecycle.imm_initial_weights.size());
    }

    Eigen::VectorXf weights(static_cast<Eigen::Index>(model_count));
    float sum = 0.0f;
    for (std::size_t i = 0; i < model_count; ++i) {
      const float value = config.lifecycle.imm_initial_weights[i];
      if (value < 0.0f || value > 1.0f) {
        AbortLifecycleAssemblyConfigViolation(
            "IMM initial weight must be in [0,1]", value);
      }
      weights(static_cast<Eigen::Index>(i)) = value;
      sum += value;
    }

    if (std::fabs(sum - 1.0f) > 1e-3f) {
      AbortLifecycleAssemblyConfigViolation(
          "IMM initial weights must sum to 1", sum);
    }
    return weights;
  }
};

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_COMPONENT_FACTORY_H_
