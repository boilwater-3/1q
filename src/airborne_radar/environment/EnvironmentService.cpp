#include "airborne_radar/environment/EnvironmentService.h"

#include "airborne_radar/environment/PropagationModel.h"
#include "airborne_radar/environment/SceneManager.h"

namespace airborne_radar {
namespace environment {
namespace {

session::EnvironmentSceneState BuildSceneStateFromModelConfig(
    const config::EnvironmentScenarioConfig& config) {
  session::EnvironmentSceneState scene_state;
  scene_state.atmospheric_physics = config.atmospheric_physics;
  scene_state.vegetation_scatter_physics =
      config.vegetation_scatter_physics;
  return scene_state;
}

}  // namespace

EnvironmentService::EnvironmentService(
    const config::EnvironmentScenarioConfig& config)
    : scene_manager_(new SceneManager(BuildSceneStateFromModelConfig(config))),
      propagation_model_(new PropagationModel()) {
  RefreshFrozenSnapshotFromActiveScene();
}

EnvironmentService::~EnvironmentService() = default;

void EnvironmentService::BeginCycle(
    const session::EnvironmentCycleContext& cycle_context) {
  current_cycle_context_ = cycle_context;
  scene_manager_->CommitPendingScene(cycle_context);
  RefreshFrozenSnapshotFromActiveScene();
}

session::EnvironmentSnapshot EnvironmentService::SampleEnvironment() const {
  return frozen_snapshot_;
}

void EnvironmentService::UpdateSceneState(
    const session::EnvironmentSceneState& scene_state) {
  scene_manager_->UpdatePendingScene(scene_state);
}

session::EnvironmentSceneState EnvironmentService::GetPendingSceneState() const {
  return scene_manager_ == nullptr
             ? session::EnvironmentSceneState{}
             : scene_manager_->GetPendingScene();
}

void EnvironmentService::UpdateModelConfig(
    const config::EnvironmentScenarioConfig& config) {
  scene_manager_->UpdatePendingScene(BuildSceneStateFromModelConfig(config));
}

EnvironmentServiceRuntimeState EnvironmentService::CaptureRuntimeState() const {
  EnvironmentServiceRuntimeState state;
  if (scene_manager_ != nullptr) {
    state.active_scene_state = scene_manager_->GetActiveScene();
    state.pending_scene_state = scene_manager_->GetPendingScene();
    state.active_cycle_context = scene_manager_->GetActiveCycleContext();
  }
  return state;
}

void EnvironmentService::RestoreRuntimeState(
    const EnvironmentServiceRuntimeState& state) {
  if (scene_manager_ != nullptr) {
    scene_manager_->RestoreState(state.active_scene_state,
                                 state.pending_scene_state,
                                 state.active_cycle_context);
  }
  current_cycle_context_ = state.active_cycle_context;
  RefreshFrozenSnapshotFromActiveScene();
}

void EnvironmentService::RefreshFrozenSnapshotFromActiveScene() {
  frozen_snapshot_ = session::EnvironmentSnapshot{};
  if (scene_manager_ == nullptr || propagation_model_ == nullptr) {
    return;
  }

  const PropagationResult propagation_result =
      propagation_model_->Evaluate(scene_manager_->GetActiveScene());
  const session::EnvironmentSceneState& active_scene =
      scene_manager_->GetActiveScene();
  frozen_snapshot_.cycle_index = current_cycle_context_.cycle_index;
  frozen_snapshot_.cycle_dt_sec = current_cycle_context_.dt_sec;
  frozen_snapshot_.propagation_loss_db =
      propagation_result.propagation_loss_db;
  frozen_snapshot_.clutter_power_db = propagation_result.clutter_power_db;
  frozen_snapshot_.atmospheric_physics = active_scene.atmospheric_physics;
  frozen_snapshot_.effective_k_factor =
      oneq::environment::ResolveEffectiveKFactor(
          active_scene.atmospheric_physics);
}

}  // namespace environment
}  // namespace airborne_radar
