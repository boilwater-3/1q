/**
 * @file EnvironmentSceneBuilder.cpp
 * @brief 实现场景状态构造器。
 */

#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"

namespace airborne_radar {
namespace environment {

JammerEmitterState BuildTypedJammerEmitter(
    JammingTechnique technique,
    const JammerEmitterParams& params) {
  JammerEmitterState emitter;
  emitter.technique = technique;
  emitter.power_db = params.power_db;
  emitter.js_db = params.js_db;
  emitter.frequency_overlap_ratio = params.frequency_overlap_ratio;
  emitter.prf_lock_risk = params.prf_lock_risk;
  emitter.in_sidelobe = params.in_sidelobe;
  emitter.azimuth_deg = params.azimuth_deg;
  emitter.elevation_deg = params.elevation_deg;
  emitter.angular_span_deg = params.angular_span_deg;
  emitter.confidence = params.confidence;
  return emitter;
}

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
    const JammerEmitterState& emitter) {
  JammerEmitterState typed_emitter = emitter;
  typed_emitter.technique = JammingTechnique::kNoiseSuppression;
  return AddJammer(typed_emitter);
}

EnvironmentSceneBuilder& EnvironmentSceneBuilder::AddDeceptionJammer(
    const JammerEmitterState& emitter) {
  JammerEmitterState typed_emitter = emitter;
  typed_emitter.technique = JammingTechnique::kDeception;
  return AddJammer(typed_emitter);
}

EnvironmentSceneBuilder& EnvironmentSceneBuilder::AddRepeaterJammer(
    const JammerEmitterState& emitter) {
  JammerEmitterState typed_emitter = emitter;
  typed_emitter.technique = JammingTechnique::kRepeater;
  return AddJammer(typed_emitter);
}

EnvironmentSceneState EnvironmentSceneBuilder::Build() const {
  return scene_state_;
}

}  // namespace environment
}  // namespace airborne_radar
