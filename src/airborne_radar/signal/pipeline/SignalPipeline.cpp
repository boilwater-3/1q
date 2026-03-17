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
#include "airborne_radar/signal/tracking/TrackLifecycleManager.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/BeamControlResolver.h"
#include "airborne_radar/signal/detection/MeasurementErrorModel.h"
#include "airborne_radar/signal/detection/SignalDetector.h"
#include "airborne_radar/signal/detection/TargetGeometryResolver.h"
#include "airborne_radar/signal/pipeline/SignalComponentFactory.h"
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
/// @param geometry 统一解析后的目标几何信息。
/// @param range_error_std 距离测量标准差（m）。
/// @param angle_error_std 等效角度测量标准差（rad）。
/// @param default_measurement_noise_std 默认回退量测标准差（m）。
/// @return 3x3 笛卡尔位置量测协方差。
/// @note angle_error_std 由 MeasurementErrorModel 基于有效 az/el 波束宽度
///       合成得到，effective beamwidth 来自 BeamControlResolver。
tracking::MeasurementCovariance BuildMeasurementCovariance(
    const detection::ResolvedTargetGeometry& geometry,
    float range_error_std,
    float angle_error_std,
    float default_measurement_noise_std) {
  if (range_error_std <= 0.0f || angle_error_std <= 0.0f) {
    return tracking::MeasurementCovariance::Identity() *
           default_measurement_noise_std * default_measurement_noise_std;
  }

  const float range_m = std::max(geometry.range_m, 0.1f);
  const float var_r = range_error_std * range_error_std;
  const float var_theta = angle_error_std * angle_error_std;
  Eigen::Vector3f pos = geometry.has_cartesian_position
                            ? geometry.position_m
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

/// @brief 约束控制比例，避免非法值污染运行时配置。
float ClampProfileScale(float scale, float fallback) {
  if (!std::isfinite(scale) || scale <= 0.0f) {
    return fallback;
  }
  return scale;
}

/// @brief 约束浮点范围，避免 profile 将配置推到非法区间。
float ClampFloat(float value, float min_value, float max_value) {
  return std::max(min_value, std::min(max_value, value));
}

/// @brief 将 dB 功率转换为线性功率（W）。
float DbToLinearPower(float power_db) {
  return std::pow(10.0f, power_db / 10.0f);
}

/// @brief 归一化 IMM 初始权重，避免 profile 调整后破坏概率约束。
void NormalizeImmInitialWeights(std::vector<float>* weights) {
  if (weights == nullptr || weights->empty()) {
    return;
  }

  float sum = 0.0f;
  for (std::size_t i = 0; i < weights->size(); ++i) {
    (*weights)[i] = ClampFloat((*weights)[i], 0.0f, 1.0f);
    sum += (*weights)[i];
  }

  if (sum <= 1e-6f) {
    const float uniform_weight = 1.0f / static_cast<float>(weights->size());
    for (std::size_t i = 0; i < weights->size(); ++i) {
      (*weights)[i] = uniform_weight;
    }
    return;
  }

  for (std::size_t i = 0; i < weights->size(); ++i) {
    (*weights)[i] /= sum;
  }
}

/// @brief 将线性比例转换为 dB 修正量。
float ToDbDelta(float linear_scale) {
  const float clamped_scale = ClampProfileScale(linear_scale, 1.0f);
  return 10.0f * std::log10(clamped_scale);
}

/// @brief 计算控制 profile 对波束宽度的收缩比例。
float ResolveBeamwidthScale(const common::RadarControlProfile& control_profile) {
  float beamwidth_scale = 1.0f;
  if (control_profile.enable_lpi_beamforming) {
    beamwidth_scale = std::min(beamwidth_scale, 0.75f);
  }
  if (control_profile.enable_adaptive_beamforming) {
    beamwidth_scale = std::min(beamwidth_scale, 0.60f);
  }
  return beamwidth_scale;
}

/// @brief 计算 profile 对经验检测项的信号修正。
float ComputeHeuristicSignalAdjustmentDb(
    const common::RadarControlProfile& control_profile) {
  float adjustment_db = 0.0f;
  if (control_profile.enable_lpi_power_control) {
    adjustment_db += ToDbDelta(control_profile.lpi_power_scale);
  }
  if (control_profile.enable_lpi_beamforming) {
    adjustment_db += 1.0f;
  }
  if (control_profile.enable_adaptive_beamforming) {
    adjustment_db += 1.5f;
  }
  return adjustment_db;
}

/// @brief 计算 profile 对经验环境惩罚项的抵消量。
float ComputeHeuristicEnvironmentReliefDb(
    const common::RadarControlProfile& control_profile,
    const environment::EnvironmentSnapshot& environment_snapshot) {
  float relief_db = 0.0f;
  if (environment_snapshot.jamming_detected) {
    if (control_profile.enable_sidelobe_canceller) {
      relief_db += environment_snapshot.jammer_in_sidelobe ? 3.0f : 0.8f;
    }
    if (control_profile.enable_agility_frequency) {
      relief_db += 0.6f + 1.8f * ClampFloat(
                                     environment_snapshot
                                         .jammer_frequency_overlap_ratio,
                                     0.0f, 1.0f);
    }
    if (control_profile.enable_eccm_rejitter) {
      relief_db += 0.4f +
                   1.4f * ClampFloat(
                              environment_snapshot.jammer_prf_lock_risk, 0.0f,
                              1.0f);
    }
  }
  if (control_profile.enable_adaptive_beamforming) {
    relief_db += environment_snapshot.jammer_in_sidelobe ? 1.2f : 0.8f;
  }
  if (control_profile.eccm_burnthrough_gain > 1.0f) {
    const float jammer_scale =
        0.5f + 0.06f * ClampFloat(environment_snapshot.jammer_power_db, 0.0f,
                                  20.0f);
    relief_db += ToDbDelta(control_profile.eccm_burnthrough_gain) *
                 ClampFloat(jammer_scale, 0.5f, 1.7f);
  }
  return relief_db;
}

/// @brief 将控制 profile 映射为当前周期生效的运行时配置。
void ApplyControlProfileToConfig(const common::RadarControlProfile& control_profile,
                                 SignalPipelineConfig* runtime_config) {
  if (runtime_config == nullptr) {
    return;
  }

  if (control_profile.enable_lpi_power_control) {
    runtime_config->detection.radar_system.transmitter.peak_power_w *=
        ClampProfileScale(control_profile.lpi_power_scale, 1.0f);
    runtime_config->tracking.kalman_measurement_noise_std *= 1.15f;
    runtime_config->association.unassigned_cost *= 0.90f;
  }

  if (control_profile.lpi_dwell_scale != 1.0f) {
    const float dwell_scale =
        std::max(0.25f, std::min(4.0f,
                                 ClampProfileScale(control_profile.lpi_dwell_scale,
                                                   1.0f)));
    const double scaled_pulse_count =
        static_cast<double>(runtime_config->detection.pulse_count) *
        static_cast<double>(dwell_scale);
    runtime_config->detection.pulse_count = std::max(
        1, static_cast<int>(std::lround(scaled_pulse_count)));
  }

  if (control_profile.enable_agility_frequency) {
    const float hop_factor =
        (control_profile.version % 2U == 0U) ? 1.015f : 0.985f;
    runtime_config->detection.radar_system.transmitter.frequency_hz *=
        hop_factor;
    runtime_config->association.unassigned_cost *= 1.25f;
    runtime_config->tracking.kalman_noise_diff_coeff *= 1.10f;
  }

  if (control_profile.enable_eccm_rejitter) {
    runtime_config->detection.radar_system.transmitter.prf_hz *= 1.10f;
    runtime_config->association.unassigned_cost *= 1.35f;
    runtime_config->tracking.kalman_noise_diff_coeff *= 1.10f;
  }

  if (control_profile.eccm_burnthrough_gain > 1.0f) {
    const float gain_db = ToDbDelta(control_profile.eccm_burnthrough_gain);
    runtime_config->detection.radar_system.receiver.noise_figure_db =
        std::max(0.0f,
                 runtime_config->detection.radar_system.receiver.noise_figure_db -
                     gain_db);
    runtime_config->association.unassigned_cost *=
        ClampFloat(control_profile.eccm_burnthrough_gain, 1.0f, 2.0f);
    runtime_config->tracking.kalman_measurement_noise_std *= 0.90f;
  }

  if (control_profile.enable_sidelobe_canceller) {
    runtime_config->detection.radar_system.antenna.enable_directional_pattern =
        true;
    runtime_config->detection.radar_system.antenna.pattern.max_sidelobe_level_db -=
        6.0f;
    runtime_config->association.unassigned_cost *= 1.10f;
    runtime_config->tracking.jamming_acceleration_penalty *= 0.60f;
  }

  const float beamwidth_scale = ResolveBeamwidthScale(control_profile);
  if (beamwidth_scale < 0.999f) {
    runtime_config->beam_control.radar_orientation.commanded_beamwidth_enabled =
        true;
    runtime_config->beam_control.radar_orientation.commanded_beamwidth_deg
        .commanded_az_beamwidth_deg = std::max(
        0.5f,
        runtime_config->detection.radar_system.antenna.nominal_az_beamwidth_deg *
            beamwidth_scale);
    runtime_config->beam_control.radar_orientation.commanded_beamwidth_deg
        .commanded_el_beamwidth_deg = std::max(
        0.5f,
        runtime_config->detection.radar_system.antenna.nominal_el_beamwidth_deg *
            beamwidth_scale);
  }

  if (control_profile.enable_adaptive_beamforming) {
    runtime_config->detection.radar_system.antenna.main_beam_gain_db += 2.0f;
    runtime_config->association.unassigned_cost *= 1.10f;
    runtime_config->tracking.kalman_measurement_noise_std *= 0.80f;
  }

  if (control_profile.enable_lpi_beamforming) {
    runtime_config->tracking.kalman_measurement_noise_std *= 0.90f;
  }

  if (control_profile.enable_sidelobe_canceller ||
      control_profile.enable_agility_frequency ||
      control_profile.enable_eccm_rejitter ||
      control_profile.eccm_burnthrough_gain > 1.0f) {
    runtime_config->tracking.speed_decay_ratio_on_loss =
        ClampFloat(runtime_config->tracking.speed_decay_ratio_on_loss + 0.05f,
                   0.0f, 0.995f);
    runtime_config->tracking.rcs_decay_ratio_on_loss =
        ClampFloat(runtime_config->tracking.rcs_decay_ratio_on_loss + 0.08f,
                   0.0f, 0.999f);
    runtime_config->tracking.jamming_acceleration_penalty =
        ClampFloat(runtime_config->tracking.jamming_acceleration_penalty * 0.70f,
                   0.0f, 1000.0f);
  }

  if (!runtime_config->lifecycle.imm_model_noise_diff_coeffs.empty()) {
    float imm_noise_scale = 1.0f;
    if (control_profile.enable_agility_frequency) {
      imm_noise_scale *= 1.15f;
    }
    if (control_profile.enable_eccm_rejitter) {
      imm_noise_scale *= 1.20f;
    }
    if (control_profile.eccm_burnthrough_gain > 1.0f) {
      imm_noise_scale *= ClampFloat(control_profile.eccm_burnthrough_gain, 1.0f,
                                    2.0f);
    }
    for (std::size_t i = 0;
         i < runtime_config->lifecycle.imm_model_noise_diff_coeffs.size(); ++i) {
      runtime_config->lifecycle.imm_model_noise_diff_coeffs[i] =
          std::max(0.001f,
                   runtime_config->lifecycle.imm_model_noise_diff_coeffs[i] *
                       imm_noise_scale);
    }
  }

  if (!runtime_config->lifecycle.imm_initial_weights.empty() &&
      runtime_config->lifecycle.imm_initial_weights.size() > 1U &&
      (control_profile.enable_agility_frequency ||
       control_profile.enable_eccm_rejitter ||
       control_profile.eccm_burnthrough_gain > 1.0f)) {
    const std::size_t last_index =
        runtime_config->lifecycle.imm_initial_weights.size() - 1U;
    const float bonus =
        control_profile.eccm_burnthrough_gain > 1.0f ? 0.18f : 0.10f;
    runtime_config->lifecycle.imm_initial_weights[last_index] += bonus;
    NormalizeImmInitialWeights(&runtime_config->lifecycle.imm_initial_weights);
  }
}

/// @brief 基于 Pipeline 配置自动装配生命周期管理器。
class AutoConfiguredLifecycleManager final
    : public tracking::ITrackLifecycleManager {
 public:
  /// @brief 使用 SignalPipeline 顶层配置构造生命周期管理器。
  /// @param config 顶层配置。
  explicit AutoConfiguredLifecycleManager(const SignalPipelineConfig& config)
      : assembly_(internal::SignalComponentFactory::BuildLifecycleAssemblyArtifacts(
            config)) {}

  /// @brief 更新生命周期状态。
  /// @param cycle 当前周期上下文。
  /// @param measurements 当前周期量测。
  void Update(const tracking::CycleContext& cycle,
              const std::vector<tracking::TrackMeasurement>& measurements)
      override {
    assembly_.lifecycle_manager->Update(cycle, measurements);
  }

  /// @brief 构建特征快照。
  /// @return 生命周期输出的轨迹特征快照。
  common::TargetFeatureList BuildFeatureSnapshot() const override {
    return assembly_.lifecycle_manager->BuildFeatureSnapshot();
  }

  /// @brief 构建供决策层消费的稳定轨迹快照。
  /// @return 生命周期输出的决策轨迹快照。
  common::DecisionTrackSnapshotList BuildDecisionSnapshot() const override {
    return assembly_.lifecycle_manager->BuildDecisionSnapshot();
  }

  /// @brief 构建完整的决策输入帧。
  common::DecisionInputFrame BuildDecisionFrame(
      std::uint32_t cycle_index, std::uint64_t batch_id,
      bool environment_jamming_detected) const override {
    return assembly_.lifecycle_manager->BuildDecisionFrame(
        cycle_index, batch_id, environment_jamming_detected);
  }

  /// @brief 构建关联种子。
  /// @return 上一周期轨迹种子列表。
  std::vector<tracking::AssociationTrackSeed> BuildAssociationSeeds()
      const override {
    return assembly_.lifecycle_manager->BuildAssociationSeeds();
  }

 private:
  internal::LifecycleAssemblyArtifacts assembly_;
};

/// @brief 单周期缓存上下文。
struct SignalCycleContext {
  const common::TargetFeatureList* input_state{nullptr};
  const environment::IEnvironmentService* environment{nullptr};
  SignalPipelineConfig runtime_config{};
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
  std::vector<detection::ResolvedTargetGeometry> target_geometry;
};

}  // namespace

/// @brief SignalPipeline 私有实现。
struct SignalPipeline::Impl {
  /// @brief 使用顶层配置构造私有实现。
  /// @param initial_config 顶层配置。
  explicit Impl(SignalPipelineConfig initial_config)
      : config(std::move(initial_config)),
        association_engine(
            internal::SignalComponentFactory::BuildAssociationConfig(config)),
        track_filter(
            internal::SignalComponentFactory::BuildTrackFilterConfig(config)) {
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
    const SignalPipelineConfig runtime_config = BuildRuntimeConfig();
    if (!runtime_config.lifecycle.enable_auto_lifecycle_manager) {
      return std::unique_ptr<tracking::ITrackLifecycleManager>();
    }
    return std::unique_ptr<tracking::ITrackLifecycleManager>(
        new AutoConfiguredLifecycleManager(runtime_config));
  }

  /// @brief 更新顶层配置。
  /// @param new_config 新配置。
  void UpdateConfig(SignalPipelineConfig new_config) {
    config = std::move(new_config);
    association_engine.UpdateConfig(
        internal::SignalComponentFactory::BuildAssociationConfig(config));
    track_filter.UpdateConfig(
        internal::SignalComponentFactory::BuildTrackFilterConfig(config));
    RebuildOwnedComponents();
  }

  /// @brief 更新当前平台姿态。
  /// @param platform_attitude_deg 平台姿态角。
  void UpdatePlatformAttitude(
      const common::PlatformAttitudeDeg& platform_attitude_deg) {
    config.beam_control.platform_attitude_deg = platform_attitude_deg;
  }

  /// @brief 获取当前平台姿态。
  /// @return 当前缓存的平台姿态角。
  common::PlatformAttitudeDeg GetPlatformAttitude() const {
    return config.beam_control.platform_attitude_deg;
  }

  /// @brief 构建本周期生效的运行时配置。
  SignalPipelineConfig BuildRuntimeConfig() const {
    SignalPipelineConfig runtime_config = config;
    ApplyControlProfileToConfig(control_profile_, &runtime_config);
    return runtime_config;
  }

  /// @brief 更新当前控制真值。
  void SetControlProfile(const common::RadarControlProfile& control_profile) {
    control_profile_ = control_profile;
  }

  /// @brief 获取当前控制真值。
  common::RadarControlProfile GetControlProfile() const {
    return control_profile_;
  }

  /// @brief 初始化单周期缓存。
  /// @param input_state 输入目标列表。
  /// @param environment 环境服务。
  void PrepareCycleContext(const common::TargetFeatureList& input_state,
                           const environment::IEnvironmentService& environment) {
    cached_context.input_state = &input_state;
    cached_context.environment = &environment;
    cached_context.runtime_config = BuildRuntimeConfig();
    cached_context.output_state = input_state;

    const std::size_t target_count = input_state.size();
    cached_context.track_measurements.clear();
    cached_context.signal_term_db.assign(target_count, 0.0f);
    cached_context.speed_penalty_db.assign(target_count, 0.0f);
    cached_context.detection_margin_db.assign(target_count, 0.0f);
    cached_context.detection_succeeded.assign(target_count, 0U);
    cached_context.association_keys.assign(target_count, 0U);
    cached_context.measurement_slots.assign(target_count, -1);
    cached_context.target_geometry.resize(target_count);
    cached_context.measurement_covariances.assign(
        target_count, tracking::MeasurementCovariance::Identity() *
                          cached_context.runtime_config.tracking
                              .kalman_measurement_noise_std *
                          cached_context.runtime_config.tracking
                              .kalman_measurement_noise_std);
    cached_context.association_result = association::AssociationResult();

    association_engine.UpdateConfig(
        internal::SignalComponentFactory::BuildAssociationConfig(
            cached_context.runtime_config));
    track_filter.UpdateConfig(
        internal::SignalComponentFactory::BuildTrackFilterConfig(
            cached_context.runtime_config));
  }

  /// @brief 采样本周期环境快照。
  void SampleEnvironment() {
    cached_context.environment_snapshot =
        cached_context.environment->SampleEnvironment();
  }

  /// @brief 运行经验探测逻辑。
  void RunHeuristicDetection() {
    const common::TargetFeatureList& input = *cached_context.input_state;
    const SignalPipelineConfig& runtime_config = cached_context.runtime_config;
    const std::size_t count = input.size();
    const float signal_adjustment_db =
        ComputeHeuristicSignalAdjustmentDb(control_profile_);
    for (std::size_t i = 0; i < count; ++i) {
      cached_context.target_geometry[i] =
          detection::TargetGeometryResolver::Resolve(input[i]);
      cached_context.signal_term_db[i] =
          input[i].current_track_rcs * 6.0f + signal_adjustment_db;
      cached_context.speed_penalty_db[i] =
          ResolveSpeedMagnitude(input[i]) * 0.002f;
    }

    const float jamming_penalty_db =
        cached_context.environment_snapshot.jamming_detected
            ? (1.0f +
               0.35f * ClampFloat(
                           cached_context.environment_snapshot.jammer_power_db,
                           0.0f, 20.0f) +
               2.0f * ClampFloat(
                           cached_context.environment_snapshot
                               .jammer_frequency_overlap_ratio,
                           0.0f, 1.0f) +
               1.5f * ClampFloat(
                           cached_context.environment_snapshot.jammer_prf_lock_risk,
                           0.0f, 1.0f) +
               (cached_context.environment_snapshot.jammer_in_sidelobe ? 1.0f
                                                                       : 0.0f))
            : 0.0f;
    const float environment_penalty_db =
        std::max(
            0.0f,
            cached_context.environment_snapshot.propagation_loss_db * 0.2f +
                cached_context.environment_snapshot.clutter_power_db * 0.3f +
                jamming_penalty_db -
                ComputeHeuristicEnvironmentReliefDb(control_profile_,
                                                    cached_context
                                                        .environment_snapshot));
    for (std::size_t i = 0; i < count; ++i) {
      const float margin = cached_context.signal_term_db[i] -
                           cached_context.speed_penalty_db[i] -
                           environment_penalty_db;
      cached_context.detection_margin_db[i] = margin;
      cached_context.detection_succeeded[i] = static_cast<std::uint8_t>(
          margin >= runtime_config.detection.min_detection_margin_db);
    }
  }

  /// @brief 运行物理探测逻辑。
  void RunPhysicalDetection() {
    const common::TargetFeatureList& input = *cached_context.input_state;
    const SignalPipelineConfig& runtime_config = cached_context.runtime_config;
    const std::size_t count = input.size();

    float clutter_w = std::pow(
        10.0f, cached_context.environment_snapshot.clutter_power_db / 10.0f);
    if (control_profile_.enable_sidelobe_canceller) {
      clutter_w *= cached_context.environment_snapshot.jammer_in_sidelobe
                       ? 0.55f
                       : 0.80f;
    }

    float jam_w = cached_context.environment_snapshot.jamming_detected
                      ? DbToLinearPower(
                            cached_context.environment_snapshot.jammer_power_db)
                      : 0.0f;
    if (control_profile_.enable_sidelobe_canceller) {
      jam_w *= cached_context.environment_snapshot.jammer_in_sidelobe
                   ? 0.35f
                   : 0.80f;
    }
    if (control_profile_.enable_agility_frequency) {
      const float overlap_ratio = ClampFloat(
          cached_context.environment_snapshot.jammer_frequency_overlap_ratio,
          0.0f, 1.0f);
      jam_w *= ClampFloat(1.0f - 0.60f * overlap_ratio, 0.25f, 1.0f);
    }
    if (control_profile_.enable_eccm_rejitter) {
      const float prf_lock_risk = ClampFloat(
          cached_context.environment_snapshot.jammer_prf_lock_risk, 0.0f, 1.0f);
      jam_w *= ClampFloat(1.0f - 0.50f * prf_lock_risk, 0.35f, 1.0f);
    }
    if (control_profile_.enable_adaptive_beamforming) {
      jam_w *= cached_context.environment_snapshot.jammer_in_sidelobe ? 0.85f
                                                                      : 0.75f;
    }

    detection::EnvironmentState env;
    env.propagation_loss_db =
        cached_context.environment_snapshot.propagation_loss_db;
    env.clutter_noise_w = clutter_w;
    env.jam_noise_w = jam_w;

    signal_detector->UpdateConfig(runtime_config.detection.radar_system);

    for (std::size_t i = 0; i < count; ++i) {
      cached_context.target_geometry[i] =
          detection::TargetGeometryResolver::Resolve(input[i]);
      detection::TargetReturn target;
      target.rcs_m2 = input[i].current_track_rcs;
      target.range_m = cached_context.target_geometry[i].range_m;
      target.swerling_type = static_cast<detection::SwerlingModel>(
          input[i].target_swerling_type);

      const detection::ResolvedBeamState beam_state =
          detection::BeamControlResolver::Resolve(
              runtime_config.detection.radar_system.antenna,
              runtime_config.beam_control.radar_orientation,
              runtime_config.beam_control.platform_attitude_deg,
              cached_context.target_geometry[i].look_angles_deg);
      const detection::DetectionResult detection_result =
          signal_detector->Detect(
              target, env, beam_state.one_way_antenna_gain_db,
              runtime_config.detection.pulse_count,
              runtime_config.detection.coherent_integration);
      const detection::MeasurementErrorState measurement_error =
          detection::MeasurementErrorModel::Compute(
              detection_result.snr_db, beam_state.effective_beamwidth_deg,
              runtime_config.detection.radar_system.transmitter.bandwidth_hz);

      cached_context.signal_term_db[i] = detection_result.snr_db;
      cached_context.speed_penalty_db[i] = 0.0f;
      cached_context.detection_margin_db[i] = detection_result.snr_db;
      cached_context.detection_succeeded[i] = static_cast<std::uint8_t>(
          detection_result.detected ? 1U : 0U);
      cached_context.measurement_covariances[i] = BuildMeasurementCovariance(
          cached_context.target_geometry[i], measurement_error.range_error_std_m,
          measurement_error.angle_error_std_rad,
          runtime_config.tracking.kalman_measurement_noise_std);
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
      measurement.raw_measurement.source_index = i;
      measurement.raw_measurement.external_target_id =
          input[i].external_target_id;
      measurement.raw_measurement.association_key =
          cached_context.association_keys[i];
      measurement.raw_measurement.matched_existing_track = match != nullptr;
      measurement.raw_measurement.association_cost =
          match != nullptr ? match->cost : 0.0f;
      measurement.raw_measurement.used_position_association =
          cached_context.association_result.used_position_association;
      measurement.raw_measurement.used_external_association_seeds =
          cached_context.association_result.used_external_association_seeds;
      measurement.raw_measurement.detection_margin_db =
          cached_context.detection_margin_db[i];
      measurement.raw_measurement.has_cartesian_position =
          cached_context.target_geometry[i].has_cartesian_position;
      measurement.raw_measurement.position =
          measurement.raw_measurement.has_cartesian_position
              ? cached_context.target_geometry[i].position_m
              : Eigen::Vector3f::Zero();
      measurement.raw_measurement.measurement_covariance =
          cached_context.measurement_covariances[i];
      measurement.filtered_feature.jamming_detected =
          cached_context.environment_snapshot.jamming_detected;

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
      measurement.filtered_feature.observed_speed =
          ResolveSpeedMagnitude(output[i]);
      measurement.filtered_feature.velocity = ResolveVelocityVector(output[i]);
      measurement.filtered_feature.observed_acceleration =
          ResolveAccelerationMagnitude(output[i]);
      measurement.filtered_feature.acceleration =
          ResolveAccelerationVector(output[i]);
      measurement.filtered_feature.rcs = output[i].current_track_rcs;
      measurement.filtered_feature.jamming_detected =
          cached_context.environment_snapshot.jamming_detected;
    }
  }

  /// @brief 收尾当前周期输出。
  void CollectOutputs() {}

  /// @brief 依据当前配置重建自持有组件。
  void RebuildOwnedComponents() {
    internal::OwnedSignalComponents components =
        internal::SignalComponentFactory::BuildOwnedPipelineComponents(config);
    kalman_predictor = std::move(components.kalman_predictor);
    kalman_updater = std::move(components.kalman_updater);
    signal_detector = std::move(components.signal_detector);
  }

  SignalPipelineConfig config{};
  common::RadarControlProfile control_profile_{};
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

void SignalPipeline::UpdatePlatformAttitude(
    const common::PlatformAttitudeDeg& platform_attitude_deg) {
  impl_->UpdatePlatformAttitude(platform_attitude_deg);
}

common::PlatformAttitudeDeg SignalPipeline::GetPlatformAttitude() const {
  return impl_->GetPlatformAttitude();
}

void SignalPipeline::SetControlProfile(
    const common::RadarControlProfile& control_profile) {
  impl_->SetControlProfile(control_profile);
}

common::RadarControlProfile SignalPipeline::GetControlProfile() const {
  return impl_->GetControlProfile();
}

void SignalPipeline::UpdateConfig(SignalPipelineConfig config) {
  impl_->UpdateConfig(std::move(config));
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
