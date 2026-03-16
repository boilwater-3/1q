// Copyright 2026. All Rights Reserved.
//
// Description: SignalPipeline 的显式步骤编排实现。

#include "1q/airborne_radar/signal/pipeline/SignalPipeline.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <numeric>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <spdlog/spdlog.h>

#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/signal/tracking/TrackLifecycleManager.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/BeamControlResolver.h"
#include "airborne_radar/signal/detection/MeasurementErrorModel.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/detection/TargetLookResolver.h"
#include "airborne_radar/signal/tracking/BoostTrackPool.h"
#include "airborne_radar/signal/tracking/KalmanPredictor.h"
#include "airborne_radar/signal/tracking/KalmanUpdater.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {

/// @brief 将内部关联质量观测指标转换为对外公开格式。
/// @param source 内部关联质量观测指标。
/// @return Pipeline 对外公开的关联质量观测指标。
AssociationQualityMetrics ToPipelineAssociationQualityMetrics(
    const association::AssociationQualityMetrics& source) {
  AssociationQualityMetrics metrics;
  metrics.prior_track_count = source.prior_track_count;
  metrics.detection_count = source.detection_count;
  metrics.matched_count = source.matched_count;
  metrics.new_track_count = source.new_track_count;
  metrics.missed_track_count = source.missed_track_count;
  metrics.match_rate = source.match_rate;
  metrics.new_track_rate = source.new_track_rate;
  metrics.missed_track_rate = source.missed_track_rate;
  metrics.mean_match_cost = source.mean_match_cost;
  metrics.p95_match_cost = source.p95_match_cost;
  return metrics;
}

/// @brief 从顶层配置构造轨迹滤波配置。
/// @param config Signal 顶层配置。
/// @return TrackFilter 使用的配置。
tracking::TrackFilterConfig ToTrackFilterConfig(
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
association::DataAssociationConfig ToAssociationConfig(
    const SignalPipelineConfig& config) {
  association::DataAssociationConfig association_config;
  association_config.kalman_noise_diff_coeff =
      config.tracking.kalman_noise_diff_coeff;
  association_config.kalman_measurement_noise_std =
      config.tracking.kalman_measurement_noise_std;
  return association_config;
}

/// @brief 在关联结果中查找指定输入目标索引对应的匹配结果。
/// @param result 关联结果。
/// @param target_index 输入目标索引。
/// @return 若命中已有轨迹则返回匹配结果指针，否则返回空指针。
const association::AssociationMatch* FindAssociationMatch(
    const association::AssociationResult& result,
    std::size_t target_index) {
  for (const association::AssociationMatch& match : result.matches) {
    if (match.target_index == target_index) {
      return &match;
    }
  }
  return nullptr;
}

/// @brief 根据距离/角度测量精度构造笛卡尔位置量测协方差。
/// @param target 目标当前位置量测。
/// @param range_error_std 距离测量标准差（m）。
/// @param angle_error_std 等效角度测量标准差（rad）。
/// @param default_measurement_noise_std 默认回退量测标准差（m）。
/// @return 3x3 笛卡尔位置量测协方差。
/// @note angle_error_std 由 MeasurementErrorModel 基于有效 az/el 波束宽度
///       合成得到，effective beamwidth 来自 BeamControlResolver。
tracking::MeasurementCovariance BuildMeasurementCovariance(
    const common::TargetFeature& target,
    float range_error_std,
    float angle_error_std,
    float default_measurement_noise_std) {
  if (range_error_std <= 0.0f || angle_error_std <= 0.0f) {
    return tracking::MeasurementCovariance::Identity() *
           default_measurement_noise_std * default_measurement_noise_std;
  }

  const float range_m = target.range_m > 0.1f
                            ? target.range_m
                            : std::max(Eigen::Vector3f(target.position_x,
                                                      target.position_y,
                                                      target.position_z)
                                           .norm(),
                                       0.1f);
  const float var_r = range_error_std * range_error_std;
  const float var_theta = angle_error_std * angle_error_std;
  Eigen::Vector3f pos =
      (target.position_x != 0.0f || target.position_y != 0.0f ||
       target.position_z != 0.0f)
          ? Eigen::Vector3f(target.position_x, target.position_y,
                            target.position_z)
          : Eigen::Vector3f(range_m, 0.0f, 0.0f);
  const float pos_norm = pos.norm();
  if (pos_norm > 0.1f) {
    const Eigen::Vector3f u = pos / pos_norm;
    const Eigen::Matrix3f identity = Eigen::Matrix3f::Identity();
    const Eigen::Matrix3f uu_t = u * u.transpose();
    return var_r * uu_t + (range_m * range_m * var_theta) * (identity - uu_t);
  }

  return tracking::MeasurementCovariance::Identity() * var_r;
}

/// @brief 解析目标标量速度。
/// @param target 输入目标。
/// @return 标量速度模长。
float ResolveSpeedMagnitude(const common::TargetFeature& target) {
  const Eigen::Vector3f velocity(target.current_track_velocity_x,
                                 target.current_track_velocity_y,
                                 target.current_track_velocity_z);
  if (velocity.squaredNorm() > 0.0f) {
    return velocity.norm();
  }
  return target.current_track_speed;
}

/// @brief 解析目标速度向量。
/// @param target 输入目标。
/// @return 速度向量。
Eigen::Vector3f ResolveVelocityVector(const common::TargetFeature& target) {
  const Eigen::Vector3f velocity(target.current_track_velocity_x,
                                 target.current_track_velocity_y,
                                 target.current_track_velocity_z);
  if (velocity.squaredNorm() > 0.0f) {
    return velocity;
  }
  return Eigen::Vector3f(target.current_track_speed, 0.0f, 0.0f);
}

/// @brief 解析目标标量加速度。
/// @param target 输入目标。
/// @return 标量加速度模长。
float ResolveAccelerationMagnitude(const common::TargetFeature& target) {
  const Eigen::Vector3f acceleration(target.current_track_acceleration_x,
                                     target.current_track_acceleration_y,
                                     target.current_track_acceleration_z);
  if (acceleration.squaredNorm() > 0.0f) {
    return acceleration.norm();
  }
  return target.current_track_acceleration;
}

/// @brief 解析目标加速度向量。
/// @param target 输入目标。
/// @return 加速度向量。
Eigen::Vector3f ResolveAccelerationVector(const common::TargetFeature& target) {
  const Eigen::Vector3f acceleration(target.current_track_acceleration_x,
                                     target.current_track_acceleration_y,
                                     target.current_track_acceleration_z);
  if (acceleration.squaredNorm() > 0.0f) {
    return acceleration;
  }
  return Eigen::Vector3f(target.current_track_acceleration, 0.0f, 0.0f);
}

/// @brief 终止生命周期自动装配配置违规。
/// @param message 错误消息。
/// @param value 附带数值。
[[noreturn]] void AbortLifecycleAssemblyConfigViolation(const char* message,
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
[[noreturn]] void AbortLifecycleAssemblyConfigViolation(const char* message,
                                                        std::size_t value) {
  spdlog::critical(
      "[SignalPipeline] lifecycle auto-assembly config violation: {} "
      "(value={})",
      message, value);
  std::abort();
}

/// @brief 基于 Pipeline 配置自动装配生命周期管理器。
class AutoConfiguredLifecycleManager final
    : public tracking::ITrackLifecycleManager {
 public:
  /// @brief 使用 SignalPipeline 顶层配置构造生命周期管理器。
  /// @param config 顶层配置。
  explicit AutoConfiguredLifecycleManager(const SignalPipelineConfig& config)
      : pool_(config.lifecycle.lifecycle_track_pool_initial_chunk,
              config.lifecycle.lifecycle_track_pool_max_chunks) {
    const float measurement_noise_std =
        std::max(config.tracking.kalman_measurement_noise_std, 0.001f);

    if (config.lifecycle.enable_imm_lifecycle) {
      const std::size_t model_count =
          config.lifecycle.imm_model_noise_diff_coeffs.size();
      if (model_count == 0U) {
        AbortLifecycleAssemblyConfigViolation(
            "IMM enabled but imm_model_noise_diff_coeffs is empty",
            model_count);
      }

      imm_predictors_owned_.reserve(model_count);
      imm_updaters_owned_.reserve(model_count);
      imm_predictors_.reserve(model_count);
      imm_updaters_.reserve(model_count);

      for (std::size_t i = 0; i < model_count; ++i) {
        const float noise_diff_coeff =
            config.lifecycle.imm_model_noise_diff_coeffs[i];
        if (noise_diff_coeff <= 0.0f) {
          AbortLifecycleAssemblyConfigViolation(
              "IMM model noise_diff_coeff must be > 0", noise_diff_coeff);
        }

        tracking::KalmanPredictorConfig predictor_config;
        predictor_config.noise_diff_coeff = noise_diff_coeff;
        std::unique_ptr<tracking::KalmanPredictor> predictor(
            new tracking::KalmanPredictor(predictor_config));

        tracking::KalmanUpdaterConfig updater_config;
        updater_config.measurement_noise_std = measurement_noise_std;
        std::unique_ptr<tracking::KalmanUpdater> updater(
            new tracking::KalmanUpdater(updater_config));

        imm_predictors_.push_back(predictor.get());
        imm_updaters_.push_back(updater.get());
        imm_predictors_owned_.push_back(std::move(predictor));
        imm_updaters_owned_.push_back(std::move(updater));
      }

      const Eigen::MatrixXf transition_probability =
          BuildTransitionProbability(config, model_count);
      const Eigen::VectorXf initial_weights =
          BuildInitialWeights(config, model_count);

      lifecycle_manager_.reset(new tracking::TrackLifecycleManager(
          pool_, config.lifecycle.lifecycle_config, imm_predictors_,
          imm_updaters_, transition_probability, initial_weights));
      return;
    }

    if (config.tracking.enable_kalman_filter) {
      tracking::KalmanPredictorConfig predictor_config;
      predictor_config.noise_diff_coeff =
          std::max(config.tracking.kalman_noise_diff_coeff, 0.001f);
      kalman_predictor_.reset(
          new tracking::KalmanPredictor(predictor_config));

      tracking::KalmanUpdaterConfig updater_config;
      updater_config.measurement_noise_std = measurement_noise_std;
      kalman_updater_.reset(new tracking::KalmanUpdater(updater_config));

      lifecycle_manager_.reset(new tracking::TrackLifecycleManager(
          pool_, config.lifecycle.lifecycle_config, kalman_predictor_.get(),
          kalman_updater_.get()));
      return;
    }

    lifecycle_manager_.reset(new tracking::TrackLifecycleManager(
        pool_, config.lifecycle.lifecycle_config));
  }

  /// @brief 更新生命周期状态。
  /// @param cycle 当前周期上下文。
  /// @param measurements 当前周期量测。
  void Update(const tracking::CycleContext& cycle,
              const std::vector<tracking::TrackMeasurement>& measurements)
      override {
    lifecycle_manager_->Update(cycle, measurements);
  }

  /// @brief 构建特征快照。
  /// @return 生命周期输出的轨迹特征快照。
  common::TargetFeatureList BuildFeatureSnapshot() const override {
    return lifecycle_manager_->BuildFeatureSnapshot();
  }

  /// @brief 构建关联种子。
  /// @return 上一周期轨迹种子列表。
  std::vector<tracking::AssociationTrackSeed> BuildAssociationSeeds()
      const override {
    return lifecycle_manager_->BuildAssociationSeeds();
  }

 private:
  /// @brief 构建 IMM 转移矩阵。
  /// @param config 顶层配置。
  /// @param model_count IMM 模型数。
  /// @return 转移概率矩阵。
  static Eigen::MatrixXf BuildTransitionProbability(
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
  static Eigen::VectorXf BuildInitialWeights(const SignalPipelineConfig& config,
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

  tracking::BoostTrackPool pool_;
  std::unique_ptr<tracking::KalmanPredictor> kalman_predictor_;
  std::unique_ptr<tracking::KalmanUpdater> kalman_updater_;
  std::vector<std::unique_ptr<tracking::KalmanPredictor> >
      imm_predictors_owned_;
  std::vector<std::unique_ptr<tracking::KalmanUpdater> > imm_updaters_owned_;
  std::vector<const tracking::IKalmanPredictor*> imm_predictors_;
  std::vector<const tracking::IKalmanUpdater*> imm_updaters_;
  std::unique_ptr<tracking::TrackLifecycleManager> lifecycle_manager_;
};

/// @brief 单周期缓存上下文。
struct SignalCycleContext {
  const common::TargetFeatureList* input_state{nullptr};
  const environment::IEnvironmentService* environment{nullptr};
  common::TargetFeatureList output_state;
  std::vector<tracking::TrackMeasurement> track_measurements;

  environment::EnvironmentSnapshot environment_snapshot{};

  std::vector<float> signal_term_db;
  std::vector<float> speed_penalty_db;
  std::vector<float> detection_margin_db;
  std::vector<std::uint8_t> detection_succeeded;
  association::AssociationResult association_result;
  std::vector<std::uint64_t> association_keys;
  std::vector<tracking::MeasurementCovariance> measurement_covariances;
  std::vector<int> measurement_slots;
};

}  // namespace

/// @brief SignalPipeline 私有实现。
struct SignalPipeline::Impl {
  /// @brief 使用顶层配置构造私有实现。
  /// @param initial_config 顶层配置。
  explicit Impl(SignalPipelineConfig initial_config)
      : config(std::move(initial_config)),
        association_engine(ToAssociationConfig(config)),
        track_filter(ToTrackFilterConfig(config)) {
    RebuildOwnedComponents();
  }

  /// @brief 执行一次信号处理周期。
  /// @param input_state 输入目标列表。
  /// @param environment 环境服务。
  /// @return 输出目标列表。
  common::TargetFeatureList RunCycle(
      const common::TargetFeatureList& input_state,
      const environment::IEnvironmentService& environment) {
    PrepareCycleContext(input_state, environment);
    SampleEnvironment();
    if (config.detection.enable_physics_detection) {
      RunPhysicalDetection();
    } else {
      RunHeuristicDetection();
    }
    RunAssociation();
    BuildTrackMeasurements();
    ApplyTrackFilter();
    CollectOutputs();
    return cached_context.output_state;
  }

  /// @brief 获取最近一次处理周期导出的跟踪量测。
  /// @return 跟踪量测列表。
  std::vector<tracking::TrackMeasurement> GetLastTrackMeasurements() const {
    return cached_context.track_measurements;
  }

  /// @brief 获取最近一次处理周期的关联质量观测指标。
  /// @return 关联质量观测指标。
  AssociationQualityMetrics GetLastAssociationQualityMetrics() const {
    return ToPipelineAssociationQualityMetrics(
        cached_context.association_result.quality_metrics);
  }

  /// @brief 设置本周期关联阶段应使用的轨迹种子。
  /// @param seeds 外部种子。
  void SetAssociationSeeds(
      const std::vector<tracking::AssociationTrackSeed>& seeds) {
    association_engine.SetAssociationSeeds(seeds);
  }

  /// @brief 清理外部 seeds 状态并恢复无先验模式。
  void ResetAssociationSeedModeToStateless() {
    association_engine.ResetAssociationSeedModeToStateless();
  }

  /// @brief 按当前配置自动装配生命周期管理器。
  /// @return 若未启用则返回空指针。
  std::unique_ptr<tracking::ITrackLifecycleManager>
  CreateAutoLifecycleManager() const {
    if (!config.lifecycle.enable_auto_lifecycle_manager) {
      return std::unique_ptr<tracking::ITrackLifecycleManager>();
    }
    return std::unique_ptr<tracking::ITrackLifecycleManager>(
        new AutoConfiguredLifecycleManager(config));
  }

  /// @brief 更新顶层配置。
  /// @param new_config 新配置。
  void UpdateConfig(SignalPipelineConfig new_config) {
    config = std::move(new_config);
    association_engine.UpdateConfig(ToAssociationConfig(config));
    track_filter.UpdateConfig(ToTrackFilterConfig(config));
    RebuildOwnedComponents();
  }

  /// @brief 初始化单周期缓存。
  /// @param input_state 输入目标列表。
  /// @param environment 环境服务。
  void PrepareCycleContext(const common::TargetFeatureList& input_state,
                           const environment::IEnvironmentService& environment) {
    cached_context.input_state = &input_state;
    cached_context.environment = &environment;
    cached_context.output_state = input_state;

    const std::size_t target_count = input_state.size();
    cached_context.track_measurements.clear();
    cached_context.signal_term_db.assign(target_count, 0.0f);
    cached_context.speed_penalty_db.assign(target_count, 0.0f);
    cached_context.detection_margin_db.assign(target_count, 0.0f);
    cached_context.detection_succeeded.assign(target_count, 0U);
    cached_context.association_keys.assign(target_count, 0U);
    cached_context.measurement_slots.assign(target_count, -1);
    cached_context.measurement_covariances.assign(
        target_count, tracking::MeasurementCovariance::Identity() *
                          config.tracking.kalman_measurement_noise_std *
                          config.tracking.kalman_measurement_noise_std);
    cached_context.association_result = association::AssociationResult();
  }

  /// @brief 采样本周期环境快照。
  void SampleEnvironment() {
    cached_context.environment_snapshot =
        cached_context.environment->SampleEnvironment();
  }

  /// @brief 运行经验探测逻辑。
  void RunHeuristicDetection() {
    const common::TargetFeatureList& input = *cached_context.input_state;
    const std::size_t count = input.size();
    for (std::size_t i = 0; i < count; ++i) {
      cached_context.signal_term_db[i] = input[i].current_track_rcs * 6.0f;
      cached_context.speed_penalty_db[i] = ResolveSpeedMagnitude(input[i]) * 0.002f;
    }

    const float environment_penalty_db =
        cached_context.environment_snapshot.propagation_loss_db * 0.2f +
        cached_context.environment_snapshot.clutter_power_db * 0.3f +
        (cached_context.environment_snapshot.jamming_detected ? 5.0f : 0.0f);
    for (std::size_t i = 0; i < count; ++i) {
      const float margin = cached_context.signal_term_db[i] -
                           cached_context.speed_penalty_db[i] -
                           environment_penalty_db;
      cached_context.detection_margin_db[i] = margin;
      cached_context.detection_succeeded[i] = static_cast<std::uint8_t>(
          margin >= config.detection.min_detection_margin_db);
    }
  }

  /// @brief 运行物理探测逻辑。
  void RunPhysicalDetection() {
    const common::TargetFeatureList& input = *cached_context.input_state;
    const std::size_t count = input.size();

    const float clutter_w = std::pow(
        10.0f, cached_context.environment_snapshot.clutter_power_db / 10.0f);
    const float jam_w =
        cached_context.environment_snapshot.jamming_detected ? 1e-12f : 0.0f;

    detection::EnvironmentState env;
    env.propagation_loss_db =
        cached_context.environment_snapshot.propagation_loss_db;
    env.clutter_noise_w = clutter_w;
    env.jam_noise_w = jam_w;

    for (std::size_t i = 0; i < count; ++i) {
      detection::TargetReturn target;
      target.rcs_m2 = input[i].current_track_rcs;
      target.range_m = input[i].range_m > 0.0f ? input[i].range_m : 50000.0f;
      target.swerling_type = static_cast<detection::SwerlingModel>(
          input[i].target_swerling_type);

      const detection::TargetLookAnglesDeg look_angles =
          detection::TargetLookResolver::Resolve(input[i]);
      const detection::ResolvedBeamState beam_state =
          detection::BeamControlResolver::Resolve(
              config.detection.radar_system.antenna,
              config.beam_control.radar_orientation, look_angles);
      const detection::DetectionResult detection_result =
          signal_detector->Detect(
              target, env, beam_state.one_way_antenna_gain_db,
              config.detection.pulse_count,
              config.detection.coherent_integration);
      const detection::MeasurementErrorState measurement_error =
          detection::MeasurementErrorModel::Compute(
              detection_result.snr_db, beam_state.effective_beamwidth_deg,
              config.detection.radar_system.transmitter.bandwidth_hz);

      cached_context.signal_term_db[i] = detection_result.snr_db;
      cached_context.speed_penalty_db[i] = 0.0f;
      cached_context.detection_margin_db[i] = detection_result.snr_db;
      cached_context.detection_succeeded[i] = static_cast<std::uint8_t>(
          detection_result.detected ? 1U : 0U);
      cached_context.measurement_covariances[i] = BuildMeasurementCovariance(
          input[i], measurement_error.range_error_std_m,
          measurement_error.angle_error_std_rad,
          config.tracking.kalman_measurement_noise_std);
    }
  }

  /// @brief 执行位置关联。
  void RunAssociation() {
    cached_context.association_result = association_engine.AssociateDetections(
        *cached_context.input_state, cached_context.detection_succeeded,
        cached_context.measurement_covariances);
    cached_context.association_keys = cached_context.association_result.target_keys;
  }

  /// @brief 构建跟踪量测骨架。
  void BuildTrackMeasurements() {
    const common::TargetFeatureList& input = *cached_context.input_state;
    const std::size_t count = input.size();
    cached_context.track_measurements.clear();

    for (std::size_t i = 0; i < count; ++i) {
      if (cached_context.detection_succeeded[i] == 0U) {
        continue;
      }

      const association::AssociationMatch* match =
          FindAssociationMatch(cached_context.association_result, i);
      tracking::TrackMeasurement measurement;
      measurement.source_index = i;
      measurement.external_target_id = input[i].external_target_id;
      measurement.association_key = cached_context.association_keys[i];
      measurement.matched_existing_track = match != nullptr;
      measurement.association_cost = match != nullptr ? match->cost : 0.0f;
      measurement.used_position_association =
          cached_context.association_result.used_position_association;
      measurement.used_external_association_seeds =
          cached_context.association_result.used_external_association_seeds;
      measurement.detection_margin_db = cached_context.detection_margin_db[i];
      measurement.has_cartesian_position =
          input[i].position_x != 0.0f || input[i].position_y != 0.0f ||
          input[i].position_z != 0.0f;
      measurement.position = measurement.has_cartesian_position
                                 ? Eigen::Vector3f(input[i].position_x,
                                                   input[i].position_y,
                                                   input[i].position_z)
                                 : Eigen::Vector3f::Zero();
      measurement.jamming_detected =
          cached_context.environment_snapshot.jamming_detected;
      measurement.measurement_covariance =
          cached_context.measurement_covariances[i];

      cached_context.measurement_slots[i] =
          static_cast<int>(cached_context.track_measurements.size());
      cached_context.track_measurements.push_back(measurement);
    }
  }

  /// @brief 执行跟踪滤波并补全量测动态属性。
  void ApplyTrackFilter() {
    const common::TargetFeatureList& input = *cached_context.input_state;
    common::TargetFeatureList& output = cached_context.output_state;
    const std::size_t count = output.size();

    for (std::size_t i = 0; i < count; ++i) {
      tracking::TrackFilterContext filter_context;
      filter_context.detection_succeeded =
          cached_context.detection_succeeded[i] != 0U;
      filter_context.jamming_detected =
          cached_context.environment_snapshot.jamming_detected;
      filter_context.detection_margin_db = cached_context.detection_margin_db[i];
      output[i] = track_filter.Filter(input[i], filter_context);

      const int measurement_slot = cached_context.measurement_slots[i];
      if (measurement_slot < 0) {
        continue;
      }

      tracking::TrackMeasurement& measurement =
          cached_context.track_measurements[static_cast<std::size_t>(
              measurement_slot)];
      measurement.observed_speed = ResolveSpeedMagnitude(output[i]);
      measurement.velocity = ResolveVelocityVector(output[i]);
      measurement.observed_acceleration = ResolveAccelerationMagnitude(output[i]);
      measurement.acceleration = ResolveAccelerationVector(output[i]);
      measurement.rcs = output[i].current_track_rcs;
    }
  }

  /// @brief 收尾当前周期输出。
  void CollectOutputs() {}

  /// @brief 依据当前配置重建自持有组件。
  void RebuildOwnedComponents() {
    if (config.tracking.enable_kalman_filter) {
      tracking::KalmanPredictorConfig predictor_config;
      predictor_config.noise_diff_coeff =
          config.tracking.kalman_noise_diff_coeff;
      kalman_predictor.reset(new tracking::KalmanPredictor(predictor_config));

      tracking::KalmanUpdaterConfig updater_config;
      updater_config.measurement_noise_std =
          config.tracking.kalman_measurement_noise_std;
      kalman_updater.reset(new tracking::KalmanUpdater(updater_config));
    } else {
      kalman_predictor.reset();
      kalman_updater.reset();
    }

    if (config.detection.enable_physics_detection) {
      signal_detector.reset(
          new detection::SignalDetector(config.detection.radar_system));
    } else {
      signal_detector.reset();
    }
  }

  SignalPipelineConfig config{};
  association::DataAssociationEngine association_engine{};
  tracking::TrackFilter track_filter{};
  std::unique_ptr<tracking::KalmanPredictor> kalman_predictor;
  std::unique_ptr<tracking::KalmanUpdater> kalman_updater;
  std::unique_ptr<detection::SignalDetector> signal_detector;
  SignalCycleContext cached_context{};
};

SignalPipeline::SignalPipeline(SignalPipelineConfig config)
    : impl_(std::unique_ptr<Impl>(new Impl(std::move(config)))) {}

SignalPipeline::~SignalPipeline() = default;

common::TargetFeatureList SignalPipeline::RunCycle(
    const common::TargetFeatureList& input_state,
    const environment::IEnvironmentService& environment) {
  return impl_->RunCycle(input_state, environment);
}

std::vector<tracking::TrackMeasurement>
SignalPipeline::GetLastTrackMeasurements() const {
  return impl_->GetLastTrackMeasurements();
}

AssociationQualityMetrics
SignalPipeline::GetLastAssociationQualityMetrics() const {
  return impl_->GetLastAssociationQualityMetrics();
}

void SignalPipeline::SetAssociationSeeds(
    const std::vector<tracking::AssociationTrackSeed>& seeds) {
  impl_->SetAssociationSeeds(seeds);
}

void SignalPipeline::ResetAssociationSeedModeToStateless() {
  impl_->ResetAssociationSeedModeToStateless();
}

std::unique_ptr<tracking::ITrackLifecycleManager>
SignalPipeline::CreateAutoLifecycleManager() const {
  return impl_->CreateAutoLifecycleManager();
}

void SignalPipeline::UpdateConfig(SignalPipelineConfig config) {
  impl_->UpdateConfig(std::move(config));
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
