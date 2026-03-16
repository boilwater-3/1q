// Copyright 2026. All Rights Reserved.
//
// Description: 定义信号处理流水线的公共配置与默认实现入口。

#ifndef AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_
#define AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_

#include <memory>
#include <vector>

#include "1q/airborne_radar/common/RadarOrientationConfig.h"
#include "1q/airborne_radar/signal/detection/RadarEquations.h"
#include "1q/airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "1q/airborne_radar/signal/tracking/TrackLifecycleManager.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

/// @brief SignalDetectionConfig 描述信号探测域配置。
struct SignalDetectionConfig {
  /// @brief 是否启用物理化信号检测（false 则使用经验检测逻辑）。
  bool enable_physics_detection{false};

  /// @brief 雷达系统物理参数（仅在 enable_physics_detection = true 时生效）。
  detection::RadarSystemConfig radar_system{};

  /// @brief 经验检测逻辑使用的探测裕量阈值（dB）。
  float min_detection_margin_db{-2.0f};

  /// @brief 驻留期的脉冲积累数。
  int pulse_count{10};

  /// @brief 是否采用相参积累（true 为相参，false 为非相参）。
  bool coherent_integration{true};
};

/// @brief SignalBeamControlConfig 描述波束控制域配置。
struct SignalBeamControlConfig {
  /// @brief 雷达方向与控制配置。
  common::RadarOrientationConfig radar_orientation{};
};

/// @brief SignalTrackingConfig 描述信号层跟踪域配置。
struct SignalTrackingConfig {
  /// @brief 是否启用 Kalman 状态估计（位置/速度）。
  bool enable_kalman_filter{true};

  /// @brief Kalman 预测器过程噪声扩散系数。
  float kalman_noise_diff_coeff{1.0f};

  /// @brief Kalman 更新器量测噪声标准差（米）。
  float kalman_measurement_noise_std{10.0f};

  /// @brief 探测失败时速度衰减比例。
  float speed_decay_ratio_on_loss{0.90f};

  /// @brief 探测失败时 RCS 衰减比例。
  float rcs_decay_ratio_on_loss{0.85f};

  /// @brief 干扰时的加速度惩罚。
  float jamming_acceleration_penalty{0.5f};

  /// @brief 稳定跟踪时的加速度增益。
  float stable_acceleration_gain{0.05f};
};

/// @brief SignalLifecycleConfig 描述生命周期域配置。
struct SignalLifecycleConfig {
  /// @brief 是否启用 Lifecycle 自动装配。
  bool enable_auto_lifecycle_manager{false};

  /// @brief 自动装配 Lifecycle 的状态机阈值配置。
  tracking::LifecycleConfig lifecycle_config{};

  /// @brief 自动装配时是否启用 IMM 多模型路径。
  bool enable_imm_lifecycle{false};

  /// @brief IMM 各模型噪声扩散系数列表（一个元素对应一个 CV 模型）。
  std::vector<float> imm_model_noise_diff_coeffs;

  /// @brief IMM 初始权重（可选；为空时使用均匀分布）。
  std::vector<float> imm_initial_weights;

  /// @brief IMM 转移矩阵（可选；行优先扁平化 N*N）。
  std::vector<float> imm_transition_probability;

  /// @brief 自动装配 Lifecycle 时对象池初始块大小。
  std::size_t lifecycle_track_pool_initial_chunk{64};

  /// @brief 自动装配 Lifecycle 时对象池最大块数量。
  std::size_t lifecycle_track_pool_max_chunks{256};
};

/// @brief SignalPipelineConfig 描述信号处理流水线顶层配置。
struct SignalPipelineConfig {
  /// @brief 探测域配置。
  SignalDetectionConfig detection{};

  /// @brief 波束控制域配置。
  SignalBeamControlConfig beam_control{};

  /// @brief 跟踪域配置。
  SignalTrackingConfig tracking{};

  /// @brief 生命周期域配置。
  SignalLifecycleConfig lifecycle{};
};

/// @brief SignalPipeline 提供可配置的信号处理周期实现。
class SignalPipeline final : public ISignalPipeline {
 public:
  /// @brief 使用配置构造信号处理模型。
  /// @param config 信号流水线顶层配置。
  explicit SignalPipeline(SignalPipelineConfig config = {});
  ~SignalPipeline() override;

  /// @brief 执行一次信号处理循环。
  /// @param input_state 本周期输入目标列表。
  /// @param environment 只读环境服务。
  /// @return 过滤后的输出目标列表。
  common::TargetFeatureList RunCycle(
      const common::TargetFeatureList& input_state,
      const environment::IEnvironmentService& environment) override;

  /// @brief 获取最近一次处理周期导出的跟踪量测。
  /// @return 跟踪量测列表。
  std::vector<tracking::TrackMeasurement>
  GetLastTrackMeasurements() const override;

  /// @brief 获取最近一次处理周期的关联质量观测指标。
  /// @return 关联质量观测指标。
  AssociationQualityMetrics GetLastAssociationQualityMetrics() const override;

  /// @brief 设置本周期关联阶段应使用的轨迹种子。
  /// @param seeds 外部注入的关联种子。
  void SetAssociationSeeds(
      const std::vector<tracking::AssociationTrackSeed>& seeds) override;

  /// @brief 清理外部 seeds 状态并恢复无先验（stateless）关联模式。
  void ResetAssociationSeedModeToStateless() override;

  /// @brief 按当前配置自动创建生命周期管理器。
  /// @return 若未启用自动装配则返回空指针；否则返回生命周期服务。
  std::unique_ptr<tracking::ITrackLifecycleManager>
  CreateAutoLifecycleManager() const override;

  /// @brief 更新信号处理配置。
  /// @param config 新配置。
  void UpdateConfig(SignalPipelineConfig config);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_
