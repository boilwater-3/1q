// Copyright 2026. All Rights Reserved.
//
// Description: 定义信号处理流水线的基础实现。

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_

#include <memory>

#include "1q/airborne_radar/signal/pipeline/ISignalPipeline.h"

namespace airborne_radar::environment {
struct EnvironmentSnapshot;
}

namespace airborne_radar::signal::pipeline {

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

  /// @brief 更新信号处理配置。
  void UpdateConfig(SignalPipelineConfig config);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace airborne_radar::signal::pipeline

#endif // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_
