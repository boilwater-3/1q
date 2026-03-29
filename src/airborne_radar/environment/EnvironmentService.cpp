#include "airborne_radar/environment/EnvironmentService.h"

#include <algorithm>
#include <utility>

#include "airborne_radar/environment/scene/SceneManager.h"
#include "airborne_radar/environment/simulation/PropagationModel.h"

namespace airborne_radar {
namespace environment {

namespace {

/**
 * @brief 将输入值裁剪到 [0, 1] 区间。
 * @param value 输入标量。
 * @return 裁剪后的结果。
 */
float Clamp01(float value) { return std::max(0.0f, std::min(1.0f, value)); }
/**
 * @brief 将输入值裁剪为非负数。
 * @param value 输入标量。
 * @return 不小于 0 的结果。
 */
float ClampNonNegative(float value) { return std::max(0.0f, value); }
/**
 * @brief 规范化场景中的单个干扰源输入。
 * @param raw_source 原始干扰源输入。
 * @return 完成边界裁剪后的干扰源状态。
 */
JammerEmitterState NormalizeEmitterState(const JammerEmitterState& raw_source) {
  JammerEmitterState normalized = raw_source;
  normalized.power_db = ClampNonNegative(raw_source.power_db);
  normalized.js_db = ClampNonNegative(raw_source.js_db);
  normalized.frequency_overlap_ratio = Clamp01(raw_source.frequency_overlap_ratio);
  normalized.prf_lock_risk = Clamp01(raw_source.prf_lock_risk);
  normalized.angular_span_deg = ClampNonNegative(raw_source.angular_span_deg);
  normalized.confidence = Clamp01(raw_source.confidence);
  return normalized;
}
/**
 * @brief 将场景干扰源转换为对外暴露的干扰事实（归一化后直接返回）。
 * @param emitter_state 场景中的干扰源状态。
 * @return 供快照输出使用的干扰事实。
 */
JammerSourceFact ToJammerSourceFact(const JammerEmitterState& emitter_state) {
  return NormalizeEmitterState(emitter_state);
}
/**
 * @brief 将环境模型配置适配为待生效场景状态。
 * @param config 环境模型配置。
 * @return 统一后的环境场景状态。
 */
EnvironmentSceneState BuildSceneStateFromModelConfig(const EnvironmentModelConfig& config) {
  EnvironmentSceneState scene_state;
  scene_state.base_propagation_loss_db = config.base_propagation_loss_db;
  scene_state.atmospheric_attenuation_db = config.atmospheric_attenuation_db;
  scene_state.terrain_reflection_db = config.terrain_reflection_db;
  scene_state.clutter_power_db = config.clutter_power_db;
  scene_state.jammer_emitters.reserve(config.jammer_sources.size());
  scene_state.jammer_emitters.insert(scene_state.jammer_emitters.end(),
                                     config.jammer_sources.begin(), config.jammer_sources.end());
  return scene_state;
}

}  // namespace

EnvironmentService::EnvironmentService(const EnvironmentModelConfig& config)
    : scene_manager_(new scene::SceneManager(BuildSceneStateFromModelConfig(config))),
      propagation_model_(new simulation::PropagationModel()) {
  RefreshFrozenSnapshotFromActiveScene();
}

EnvironmentService::~EnvironmentService() = default;

void EnvironmentService::BeginCycle(const EnvironmentCycleContext& cycle_context) {
  current_cycle_context_ = cycle_context;
  scene_manager_->CommitPendingScene(cycle_context);
  RefreshFrozenSnapshotFromActiveScene();
}

EnvironmentSnapshot EnvironmentService::SampleEnvironment() const { return frozen_snapshot_; }

void EnvironmentService::UpdateSceneState(const EnvironmentSceneState& scene_state) {
  scene_manager_->UpdatePendingScene(scene_state);
}

void EnvironmentService::UpdateModelConfig(const EnvironmentModelConfig& config) {
  scene_manager_->UpdatePendingScene(BuildSceneStateFromModelConfig(config));
}

void EnvironmentService::SetJammingDetectionThresholdDb(float threshold_db) {
  jamming_detection_threshold_db_ = threshold_db;
  RefreshFrozenSnapshotFromActiveScene();
}

void EnvironmentService::RefreshFrozenSnapshotFromActiveScene() {
  frozen_snapshot_ = EnvironmentSnapshot();
  if (scene_manager_ == nullptr || propagation_model_ == nullptr) {
    return;
  }

  const simulation::PropagationResult propagation_result =
      propagation_model_->Evaluate(scene_manager_->GetActiveScene());
  frozen_snapshot_.cycle_dt_sec = current_cycle_context_.dt_sec;
  frozen_snapshot_.propagation_loss_db = propagation_result.propagation_loss_db;
  frozen_snapshot_.clutter_power_db = propagation_result.clutter_power_db;

  const EnvironmentSceneState& active_scene = scene_manager_->GetActiveScene();
  frozen_snapshot_.jammer_sources.clear();
  frozen_snapshot_.jammer_sources.reserve(active_scene.jammer_emitters.size());
  for (std::size_t i = 0; i < active_scene.jammer_emitters.size(); ++i) {
    frozen_snapshot_.jammer_sources.push_back(ToJammerSourceFact(active_scene.jammer_emitters[i]));
  }

  frozen_snapshot_.jamming_detected =
      std::find_if(frozen_snapshot_.jammer_sources.begin(), frozen_snapshot_.jammer_sources.end(),
                   [this](const JammerSourceFact& source) {
                     return source.power_db >= jamming_detection_threshold_db_;
                   }) != frozen_snapshot_.jammer_sources.end();
}

}  // namespace environment
}  // namespace airborne_radar
