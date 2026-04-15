#include "1q/airborne_radar/environment/EnvironmentSceneBuilder.h"

namespace airborne_radar {
namespace environment {

EnvironmentSceneBuilder& EnvironmentSceneBuilder::SetAtmosphericPhysicsConfig(
    const AtmosphericPhysicsConfig& config) {
  scene_state_.atmospheric_physics = config;
  return *this;
}

EnvironmentSceneBuilder& EnvironmentSceneBuilder::SetVegetationScatterPhysicsConfig(
    const VegetationScatterPhysicsConfig& config) {
  scene_state_.vegetation_scatter_physics = config;
  return *this;
}

EnvironmentSceneBuilder& EnvironmentSceneBuilder::AddJammer(const JammerEmitterState& emitter) {
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

EnvironmentSceneState EnvironmentSceneBuilder::Build() const { return scene_state_; }

}  // namespace environment
}  // namespace airborne_radar
