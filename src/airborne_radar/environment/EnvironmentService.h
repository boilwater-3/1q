// Copyright 2026. All Rights Reserved.
//
// @file EnvironmentService.h
// @brief 定义环境建模层的基础实现。

#ifndef AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SERVICE_H_
#define AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SERVICE_H_

#include <cstddef>
#include <memory>
#include "1q/airborne_radar/environment/IEnvironmentService.h"
#include "1q/airborne_radar/environment/EnvironmentTypes.h"

namespace airborne_radar {
namespace environment {

namespace scene {
class SceneManager;
}

namespace simulation {
class PropagationModel;
}

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
