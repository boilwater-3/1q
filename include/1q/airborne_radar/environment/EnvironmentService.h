// Copyright 2026. All Rights Reserved.
//
// @file EnvironmentService.h
// @brief 定义环境建模层的基础实现。

#ifndef AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SERVICE_H_
#define AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SERVICE_H_

#include <cstddef>
#include <memory>
#include <vector>

#include "1q/airborne_radar/environment/IEnvironmentService.h"

namespace airborne_radar {
namespace environment {

namespace scene {
class SceneManager;
}

namespace simulation {
class PropagationModel;
}

/// @brief JammerEmitterState 描述场景中的单个干扰源输入。
struct JammerEmitterState {
  /// @brief 干扰技术类型。
  JammingTechnique technique{JammingTechnique::kUnknown};
  /// @brief 干扰功率估计（单位：dB）。
  float power_db{0.0f};
  /// @brief 干扰与信号比估计（单位：dB）。
  float js_db{0.0f};
  /// @brief 干扰与当前工作频率的重叠度，范围 [0, 1]。
  float frequency_overlap_ratio{0.0f};
  /// @brief 干扰对当前 PRF 锁定的风险度，范围 [0, 1]。
  float prf_lock_risk{0.0f};
  /// @brief 干扰来向方位角（单位：deg）。
  float azimuth_deg{0.0f};
  /// @brief 干扰来向俯仰角（单位：deg）。
  float elevation_deg{0.0f};
  /// @brief 干扰角域宽度（单位：deg）。
  float angular_span_deg{0.0f};
  /// @brief 干扰是否主要经由旁瓣进入。
  bool in_sidelobe{false};
  /// @brief 干扰事实置信度，范围 [0, 1]。
  float confidence{1.0f};
};

/// @brief 场景中的干扰源列表。
typedef std::vector<JammerEmitterState> JammerEmitterStateList;

/// @brief EnvironmentSceneState 描述环境层待冻结的场景状态。
struct EnvironmentSceneState {
  /// @brief 基础传播损耗（dB）。
  float base_propagation_loss_db{4.0f};

  /// @brief 大气附加衰减（dB）。
  float atmospheric_attenuation_db{1.5f};

  /// @brief 地形/多径附加项（dB）。
  float terrain_reflection_db{1.0f};

  /// @brief 杂波功率（dB）。
  float clutter_power_db{3.0f};

  /// @brief 当前场景中的干扰源输入。
  JammerEmitterStateList jammer_emitters{};
};

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

  /// @brief 多源干扰事实输入。
  JammerSourceFactList jammer_sources{};
};

/// @brief EnvironmentService 提供可配置的环境快照采样。
class EnvironmentService final : public IEnvironmentService {
public:
  /// @brief 使用配置构造环境模型。
  explicit EnvironmentService(EnvironmentModelConfig config = {});
  ~EnvironmentService() override;

  /// @brief 冻结当前周期环境事实。
  void BeginCycle(const EnvironmentCycleContext& cycle_context) override;

  /// @brief 采样并返回当前处理周期的环境条件。
  EnvironmentSnapshot SampleEnvironment() const override;

  /// @brief 更新待生效场景状态。
  void UpdateSceneState(const EnvironmentSceneState& scene_state);

  /// @brief 更新环境模型配置。
  void UpdateModelConfig(EnvironmentModelConfig config);

  /// @brief 设置干扰功率估计。
  void SetJammerPowerDb(float jammer_power_db);

  /// @brief 设置干扰判定阈值。
  void SetJammingDetectionThresholdDb(float threshold_db);

private:
  static constexpr std::size_t kNoLegacyJammerEmitterIndex =
      static_cast<std::size_t>(-1);

  void RefreshFrozenSnapshotFromActiveScene();

  std::unique_ptr<scene::SceneManager> scene_manager_;
  std::unique_ptr<simulation::PropagationModel> propagation_model_;
  EnvironmentSnapshot frozen_snapshot_{};
  EnvironmentCycleContext current_cycle_context_{};
  std::size_t pending_legacy_jammer_emitter_index_{
      kNoLegacyJammerEmitterIndex};
  std::size_t active_legacy_jammer_emitter_index_{
      kNoLegacyJammerEmitterIndex};
  float jamming_detection_threshold_db_{6.0f};
};

} // namespace environment
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SERVICE_H_
