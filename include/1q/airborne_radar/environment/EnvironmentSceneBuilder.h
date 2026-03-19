// Copyright 2026. All Rights Reserved.
//
// @file EnvironmentSceneBuilder.h
// @brief 定义面向外部调用方的环境场景构造器。

#ifndef AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SCENE_BUILDER_H_
#define AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SCENE_BUILDER_H_

#include "1q/airborne_radar/environment/EnvironmentTypes.h"

namespace airborne_radar {
namespace environment {

/// @brief EnvironmentSceneBuilder 用于构造待生效的环境场景状态。
class EnvironmentSceneBuilder {
public:
  /// @brief 默认构造函数，使用 EnvironmentSceneState 默认值初始化。
  EnvironmentSceneBuilder() = default;

  /// @brief 设置基础传播损耗。
  EnvironmentSceneBuilder& SetBasePropagationLossDb(float base_loss_db);

  /// @brief 设置大气附加衰减。
  EnvironmentSceneBuilder& SetAtmosphericAttenuationDb(
      float atmospheric_loss_db);

  /// @brief 设置地形/多径附加项。
  EnvironmentSceneBuilder& SetTerrainReflectionDb(float terrain_loss_db);

  /// @brief 设置杂波功率。
  EnvironmentSceneBuilder& SetClutterPowerDb(float clutter_power_db);

  /// @brief 追加一个完整的干扰源状态。
  EnvironmentSceneBuilder& AddJammer(const JammerEmitterState& emitter);

  /// @brief 追加一个压制式/噪声式干扰源。
  EnvironmentSceneBuilder& AddNoiseJammer(
      float power_db,
      float js_db = 0.0f,
      float frequency_overlap_ratio = 0.0f,
      float prf_lock_risk = 0.0f,
      bool in_sidelobe = false,
      float azimuth_deg = 0.0f,
      float elevation_deg = 0.0f,
      float angular_span_deg = 0.0f,
      float confidence = 1.0f);

  /// @brief 追加一个欺骗式干扰源。
  EnvironmentSceneBuilder& AddDeceptionJammer(
      float power_db,
      float js_db = 0.0f,
      float frequency_overlap_ratio = 0.0f,
      float prf_lock_risk = 0.0f,
      bool in_sidelobe = false,
      float azimuth_deg = 0.0f,
      float elevation_deg = 0.0f,
      float angular_span_deg = 0.0f,
      float confidence = 1.0f);

  /// @brief 追加一个转发式/重复器式干扰源。
  EnvironmentSceneBuilder& AddRepeaterJammer(
      float power_db,
      float js_db = 0.0f,
      float frequency_overlap_ratio = 0.0f,
      float prf_lock_risk = 0.0f,
      bool in_sidelobe = false,
      float azimuth_deg = 0.0f,
      float elevation_deg = 0.0f,
      float angular_span_deg = 0.0f,
      float confidence = 1.0f);

  /// @brief 生成当前构造结果。
  EnvironmentSceneState Build() const;

private:
  EnvironmentSceneState scene_state_{};
};

} // namespace environment
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_ENVIRONMENT_ENVIRONMENT_SCENE_BUILDER_H_
