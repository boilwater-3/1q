#include "airborne_radar/environment/SceneManager.h"

namespace airborne_radar {
namespace environment {

SceneManager::SceneManager(const EnvironmentSceneState& initial_scene)
    : active_scene_(initial_scene), pending_scene_(initial_scene) {}

void SceneManager::UpdatePendingScene(const EnvironmentSceneState& scene_state) {
  pending_scene_ = scene_state;
}

const EnvironmentSceneState& SceneManager::GetPendingScene() const { return pending_scene_; }

EnvironmentSceneState* SceneManager::MutablePendingScene() { return &pending_scene_; }

void SceneManager::CommitPendingScene(const EnvironmentCycleContext& cycle_context) {
  active_scene_ = pending_scene_;
  active_cycle_context_ = cycle_context;
}

const EnvironmentSceneState& SceneManager::GetActiveScene() const { return active_scene_; }

const EnvironmentCycleContext& SceneManager::GetActiveCycleContext() const {
  return active_cycle_context_;
}

}  // namespace environment
}  // namespace airborne_radar
