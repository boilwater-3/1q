// Copyright 2026. All Rights Reserved.
//
// Description: 定义环境建模层的基础实现。

#ifndef AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SERVICE_H_
#define AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SERVICE_H_

#include "1q/airborne_radar/environment/IEnvironmentService.h"

namespace airborne_radar {
namespace environment {

/// @brief EnvironmentModelConfig 描述环境模型参数。
struct EnvironmentModelConfig {
  /// @brief 基础传播损耗（dB）。
  float base_propagation_loss_db{4.0f};

  /// @brief 大气附加衰减（dB）。
  float atmospheric_attenuation_db{1.5f};

  /// @brief 地形/多径附加项（dB）。
  float terrain_reflection_db{1.0f};

  /// @brief 杂波功率（dB）。
  float clutter_power_db{3.0f};

  /// @brief 干扰功率估计（dB）。
  float jammer_power_db{0.0f};

  /// @brief 干扰与当前工作频率的重叠度，范围 [0, 1]。
  float jammer_frequency_overlap_ratio{0.0f};

  /// @brief 干扰对当前 PRF 锁定的风险度，范围 [0, 1]。
  float jammer_prf_lock_risk{0.0f};

  /// @brief 干扰是否主要经由旁瓣进入。
  bool jammer_in_sidelobe{false};
};

/// @brief EnvironmentService 提供可配置的环境快照采样。
class EnvironmentService final : public IEnvironmentService {
public:
  /// @brief 使用配置构造环境模型。
  explicit EnvironmentService(EnvironmentModelConfig config = {});

  /// @brief 采样并返回当前处理周期的环境条件。
  EnvironmentSnapshot SampleEnvironment() const override;

  /// @brief 更新环境模型配置。
  void UpdateModelConfig(EnvironmentModelConfig config);

  /// @brief 设置干扰功率估计。
  void SetJammerPowerDb(float jammer_power_db);

  /// @brief 设置干扰判定阈值。
  void SetJammingDetectionThresholdDb(float threshold_db);

private:
  EnvironmentModelConfig config_{};
  float jamming_detection_threshold_db_{6.0f};
};

} // namespace environment
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SERVICE_H_
