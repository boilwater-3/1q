// Copyright 2026. All Rights Reserved.
//
// Description: 定义信号处理流水线的基础实现。

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_

#include <memory>

#include "1q/airborne_radar/signal/pipeline/ISignalPipeline.h"
#include "airborne_radar/signal/detection/RadarEquations.h"

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

  /// @brief 是否启用内部关联 history fallback 兼容缓存（默认关闭）。
  bool enable_internal_association_history_fallback{false};

  /// @brief 是否允许通过运行时接口开启内部 fallback 兼容缓存（默认不允许）。
  bool allow_runtime_enable_internal_association_history_fallback{false};

  /// @brief 雷达系统物理参数（仅在 enable_physics_detection = true 时生效）。
  detection::RadarSystemConfig radar_system{};

  /// @brief 驻留期的脉冲积累数。
  int pulse_count{10};

  /// @brief 是否采用相参积累（true为相参，false为非相参）。
  bool coherent_integration{true};
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

	/// @brief 启用或关闭内部关联 history fallback 兼容模式。
  void SetInternalAssociationHistoryFallbackEnabled(bool enabled) override;

  /// @brief 清理外部 seeds 状态并恢复 fallback-history 关联模式。
  void ResetAssociationSeedModeToFallbackHistory() override;

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
