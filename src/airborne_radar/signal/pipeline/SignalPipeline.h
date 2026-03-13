// Copyright 2026. All Rights Reserved.
//
// Description: 定义信号处理流水线的基础实现。

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_

#include <memory>
#include <vector>

#include "1q/airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "airborne_radar/signal/detection/RadarEquations.h"
#include "1q/airborne_radar/signal/tracking/TrackLifecycleManager.h"

namespace airborne_radar {
namespace environment {
struct EnvironmentSnapshot;
}
}

namespace airborne_radar {
namespace signal {
namespace pipeline {

/// @brief SignalPipelineConfig 描述信号处理模型参数。
struct SignalPipelineConfig {
  /// @brief 探测裕量阈值（dB）。
  float min_detection_margin_db{-2.0f};

  /// @brief 探测失败时速度衰减比例。
  float speed_decay_ratio_on_loss{0.90f};

  /// @brief 探测失败时 RCS 衰减比例。
  float rcs_decay_ratio_on_loss{0.85f};

  /// @brief 干扰时的加速度惩罚。
  float jamming_acceleration_penalty{0.5f};

  /// @brief 稳定跟踪时的加速度增益。
  float stable_acceleration_gain{0.05f};

  /// @brief 是否启用 Kalman 状态估计（位置/速度）。
  bool enable_kalman_filter{true};

  /// @brief Kalman 预测器过程噪声扩散系数。
  float kalman_noise_diff_coeff{1.0f};

  /// @brief Kalman 更新器量测噪声标准差（米）。
  float kalman_measurement_noise_std{10.0f};

  /// @brief 是否启用物理化信号检测（false 则使用旧占位符逻辑）。
  bool enable_physics_detection{false};

  /// @brief 雷达系统物理参数（仅在 enable_physics_detection = true 时生效）。
  detection::RadarSystemConfig radar_system{};

  /// @brief 驻留期的脉冲积累数。
  int pulse_count{10};

  /// @brief 是否采用相参积累（true为相参，false为非相参）。
  bool coherent_integration{true};

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

/// @brief SignalPipeline 提供可配置的信号处理周期实现。
class SignalPipeline final : public ISignalPipeline {
public:
  /// @brief 使用配置构造信号处理模型。
  explicit SignalPipeline(SignalPipelineConfig config = {});
  ~SignalPipeline() override;

  /// @brief 执行一次信号处理循环。
  common::TargetFeatureList RunCycle(
      const common::TargetFeatureList &input_state,
      const environment::IEnvironmentService &environment) override;

  std::vector<tracking::TrackMeasurement>
  GetLastTrackMeasurements() const override;

  /// @brief 获取最近一次处理周期的关联质量观测指标。
  AssociationQualityMetrics
  GetLastAssociationQualityMetrics() const override;

  /// @brief 设置本周期关联阶段应使用的轨迹种子。
  void SetAssociationSeeds(
      const std::vector<tracking::AssociationTrackSeed> &seeds) override;

  /// @brief 清理外部 seeds 状态并恢复无先验（stateless）关联模式。
  void ResetAssociationSeedModeToStateless() override;

  /// @brief 按当前配置自动创建生命周期管理器。
  std::unique_ptr<tracking::ITrackLifecycleManager>
  CreateAutoLifecycleManager() const override;

  /// @brief 更新信号处理配置。
  void UpdateConfig(SignalPipelineConfig config);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace pipeline
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_
