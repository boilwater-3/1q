// Copyright 2026. All Rights Reserved.
//
// @file EnvironmentSceneBuilder.cpp
// @brief 实现场景状态构造器。

#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"

namespace airborne_radar {
namespace environment {

namespace {

/// @brief 构造指定技术类型的干扰源输入。
/// @param technique 干扰技术类型。
/// @param power_db 干扰功率（dB）。
/// @param js_db 干扰与信号比（dB）。
/// @param frequency_overlap_ratio 频率重叠度。
/// @param prf_lock_risk PRF 锁定风险。
/// @param in_sidelobe 是否经由旁瓣进入。
/// @param azimuth_deg 来向方位角（度）。
/// @param elevation_deg 来向俯仰角（度）。
/// @param angular_span_deg 角域宽度（度）。
/// @param confidence 干扰置信度。
/// @return 干扰源状态。
JammerEmitterState BuildTypedJammerEmitter(
    JammingTechnique technique,
    float power_db,
    float js_db,
    float frequency_overlap_ratio,
    float prf_lock_risk,
    bool in_sidelobe,
    float azimuth_deg,
    float elevation_deg,
    float angular_span_deg,
    float confidence) {
  JammerEmitterState emitter;
  emitter.technique = technique;
  emitter.power_db = power_db;
  emitter.js_db = js_db;
  emitter.frequency_overlap_ratio = frequency_overlap_ratio;
  emitter.prf_lock_risk = prf_lock_risk;
  emitter.in_sidelobe = in_sidelobe;
  emitter.azimuth_deg = azimuth_deg;
  emitter.elevation_deg = elevation_deg;
  emitter.angular_span_deg = angular_span_deg;
  emitter.confidence = confidence;
  return emitter;
}

} // namespace

EnvironmentSceneBuilder& EnvironmentSceneBuilder::SetBasePropagationLossDb(
    float base_loss_db) {
  scene_state_.base_propagation_loss_db = base_loss_db;
  return *this;
}

EnvironmentSceneBuilder& EnvironmentSceneBuilder::SetAtmosphericAttenuationDb(
    float atmospheric_loss_db) {
  scene_state_.atmospheric_attenuation_db = atmospheric_loss_db;
  return *this;
}

EnvironmentSceneBuilder& EnvironmentSceneBuilder::SetTerrainReflectionDb(
    float terrain_loss_db) {
  scene_state_.terrain_reflection_db = terrain_loss_db;
  return *this;
}

EnvironmentSceneBuilder& EnvironmentSceneBuilder::SetClutterPowerDb(
    float clutter_power_db) {
  scene_state_.clutter_power_db = clutter_power_db;
  return *this;
}

EnvironmentSceneBuilder& EnvironmentSceneBuilder::AddJammer(
    const JammerEmitterState& emitter) {
  scene_state_.jammer_emitters.push_back(emitter);
  return *this;
}

EnvironmentSceneBuilder& EnvironmentSceneBuilder::AddNoiseJammer(
    float power_db,
    float js_db,
    float frequency_overlap_ratio,
    float prf_lock_risk,
    bool in_sidelobe,
    float azimuth_deg,
    float elevation_deg,
    float angular_span_deg,
    float confidence) {
  return AddJammer(BuildTypedJammerEmitter(
      JammingTechnique::kNoiseSuppression, power_db, js_db,
      frequency_overlap_ratio, prf_lock_risk, in_sidelobe, azimuth_deg,
      elevation_deg, angular_span_deg, confidence));
}

EnvironmentSceneBuilder& EnvironmentSceneBuilder::AddDeceptionJammer(
    float power_db,
    float js_db,
    float frequency_overlap_ratio,
    float prf_lock_risk,
    bool in_sidelobe,
    float azimuth_deg,
    float elevation_deg,
    float angular_span_deg,
    float confidence) {
  return AddJammer(BuildTypedJammerEmitter(
      JammingTechnique::kDeception, power_db, js_db, frequency_overlap_ratio,
      prf_lock_risk, in_sidelobe, azimuth_deg, elevation_deg, angular_span_deg,
      confidence));
}

EnvironmentSceneBuilder& EnvironmentSceneBuilder::AddRepeaterJammer(
    float power_db,
    float js_db,
    float frequency_overlap_ratio,
    float prf_lock_risk,
    bool in_sidelobe,
    float azimuth_deg,
    float elevation_deg,
    float angular_span_deg,
    float confidence) {
  return AddJammer(BuildTypedJammerEmitter(
      JammingTechnique::kRepeater, power_db, js_db, frequency_overlap_ratio,
      prf_lock_risk, in_sidelobe, azimuth_deg, elevation_deg, angular_span_deg,
      confidence));
}

EnvironmentSceneState EnvironmentSceneBuilder::Build() const {
  return scene_state_;
}

} // namespace environment
} // namespace airborne_radar
