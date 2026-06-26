#include "airborne_radar/environment/EnvironmentService.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "airborne_radar/environment/PropagationModel.h"
#include "airborne_radar/environment/SceneManager.h"
#include "airborne_radar/utils/MathUtils.h"

namespace airborne_radar {
namespace environment {

// Using declarations for types migrated to config:: and session::
using config::EnvironmentModelConfig;
using config::JammerEmitterState;
using config::JammingSensitivityProfile;
using config::JammingTechnique;
using session::EnvironmentCycleContext;
using session::EnvironmentSceneState;
using session::EnvironmentSnapshot;
using session::JammerDirectionDeg;
using session::JammerSourceFact;

namespace {

float WrapAzimuthDeg(float azimuth_deg) {
  float wrapped = std::fmod(azimuth_deg + 180.0f, 360.0f);
  if (wrapped < 0.0f) {
    wrapped += 360.0f;
  }
  return wrapped - 180.0f;
}

bool HasExternalDirection(const JammerEmitterState& source) { return source.has_direction_deg; }

bool DeriveInSidelobeWithoutDirection(const JammerSourceFact& source) {
  float technique_bias = 0.40f;
  switch (source.technique) {
    case JammingTechnique::kNoiseSuppression:
      technique_bias = 0.58f;
      break;
    case JammingTechnique::kDeception:
      technique_bias = 0.36f;
      break;
    case JammingTechnique::kRepeater:
      technique_bias = 0.44f;
      break;
    default:
      technique_bias = 0.40f;
      break;
  }

  const float angular_focus =
      utils::ClampFloat(1.0f - source.angular_span_deg / 120.0f, 0.0f, 1.0f);
  const float confidence = utils::ClampFloat(source.confidence, 0.0f, 1.0f);
  const float js_ratio = utils::ClampFloat(source.js_db / 12.0f, 0.0f, 1.0f);
  const float sidelobe_score =
      technique_bias + 0.24f * angular_focus + 0.20f * confidence - 0.18f * js_ratio;
  return sidelobe_score >= 0.50f;
}

bool DeriveInSidelobe(const JammerSourceFact& source) {
  if (!source.has_direction_deg) {
    return DeriveInSidelobeWithoutDirection(source);
  }

  const float abs_azimuth_deg = std::fabs(source.direction_deg.azimuth_deg);
  const float abs_elevation_deg = std::fabs(source.direction_deg.elevation_deg);
  const bool inside_frontlobe =
      abs_azimuth_deg <= 12.0f && abs_elevation_deg <= 6.0f && source.angular_span_deg <= 24.0f;
  return !inside_frontlobe;
}

float DeriveFrequencyOverlapRatio(const JammerSourceFact& source) {
  float technique_bias = 0.35f;
  switch (source.technique) {
    case JammingTechnique::kNoiseSuppression:
      technique_bias = 0.55f;
      break;
    case JammingTechnique::kDeception:
      technique_bias = 0.78f;
      break;
    case JammingTechnique::kRepeater:
      technique_bias = 0.66f;
      break;
    default:
      technique_bias = 0.35f;
      break;
  }

  const float js_ratio = utils::ClampFloat(source.js_db / 12.0f, 0.0f, 1.0f);
  const float confidence = utils::ClampFloat(source.confidence, 0.0f, 1.0f);
  const float directional_focus =
      utils::ClampFloat(1.0f - source.angular_span_deg / 120.0f, 0.0f, 1.0f);
  const float sidelobe_penalty = source.in_sidelobe ? 0.12f : 0.0f;
  const float overlap = technique_bias + 0.22f * js_ratio + 0.18f * confidence +
                        0.14f * directional_focus - sidelobe_penalty;
  return utils::ClampFloat(overlap, 0.0f, 1.0f);
}

float DerivePrfLockRisk(const JammerSourceFact& source) {
  float technique_bias = 0.26f;
  switch (source.technique) {
    case JammingTechnique::kNoiseSuppression:
      technique_bias = 0.24f;
      break;
    case JammingTechnique::kDeception:
      technique_bias = 0.72f;
      break;
    case JammingTechnique::kRepeater:
      technique_bias = 0.82f;
      break;
    default:
      technique_bias = 0.26f;
      break;
  }

  const float power_ratio = utils::ClampFloat(source.power_db / 60.0f, 0.0f, 1.0f);
  const float confidence = utils::ClampFloat(source.confidence, 0.0f, 1.0f);
  const float frontlobe_bonus = source.in_sidelobe ? -0.08f : 0.08f;
  const float risk = technique_bias + 0.26f * power_ratio + 0.18f * confidence + frontlobe_bonus;
  return utils::ClampFloat(risk, 0.0f, 1.0f);
}

/**
 * @brief 规范化场景中的单个干扰源输入。
 * @param raw_source 原始干扰源输入。
 * @return 完成边界裁剪后的干扰源状态。
 */
JammerSourceFact NormalizeEmitterState(const JammerEmitterState& raw_source) {
  JammerSourceFact normalized;
  normalized.technique = raw_source.technique;
  normalized.power_db =
      utils::ClampFloat(raw_source.power_db, 0.0f, std::numeric_limits<float>::max());
  normalized.js_db = utils::ClampFloat(raw_source.js_db, 0.0f, std::numeric_limits<float>::max());
  normalized.angular_span_deg =
      utils::ClampFloat(raw_source.angular_span_deg, 0.0f, std::numeric_limits<float>::max());
  normalized.confidence = utils::ClampFloat(raw_source.confidence, 0.0f, 1.0f);
  if (HasExternalDirection(raw_source)) {
    normalized.has_direction_deg = true;
    normalized.direction_deg.azimuth_deg = WrapAzimuthDeg(raw_source.azimuth_deg);
    normalized.direction_deg.elevation_deg =
        utils::ClampFloat(raw_source.elevation_deg, -20.0f, 80.0f);
  } else {
    normalized.has_direction_deg = false;
  }
  normalized.in_sidelobe = DeriveInSidelobe(normalized);
  normalized.frequency_overlap_ratio = DeriveFrequencyOverlapRatio(normalized);
  normalized.prf_lock_risk = DerivePrfLockRisk(normalized);
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
  scene_state.atmospheric_physics = config.atmospheric_physics;
  scene_state.atmospheric_context = config.atmospheric_context;
  scene_state.vegetation_scatter_physics = config.vegetation_scatter_physics;
  scene_state.jammer_emitters.reserve(config.jammer_sources.size());
  scene_state.jammer_emitters.insert(scene_state.jammer_emitters.end(),
                                     config.jammer_sources.begin(), config.jammer_sources.end());
  return scene_state;
}

}  // namespace

EnvironmentService::EnvironmentService(const EnvironmentModelConfig& config)
    : scene_manager_(new SceneManager(BuildSceneStateFromModelConfig(config))),
      propagation_model_(new PropagationModel()) {
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

EnvironmentSceneState EnvironmentService::GetPendingSceneState() const {
  if (scene_manager_ == nullptr) {
    return EnvironmentSceneState();
  }
  return scene_manager_->GetPendingScene();
}

void EnvironmentService::UpdateModelConfig(const EnvironmentModelConfig& config) {
  scene_manager_->UpdatePendingScene(BuildSceneStateFromModelConfig(config));
}

void EnvironmentService::SetJammingSensitivityProfile(JammingSensitivityProfile profile) {
  jamming_sensitivity_profile_ = profile;
  effective_jamming_detection_threshold_db_ = ResolveJammingDetectionThresholdDb(profile);
  RefreshFrozenSnapshotFromActiveScene();
}

environment::EnvironmentServiceRuntimeState EnvironmentService::CaptureRuntimeState() const {
  environment::EnvironmentServiceRuntimeState state;
  if (scene_manager_ != nullptr) {
    state.active_scene_state = scene_manager_->GetActiveScene();
    state.pending_scene_state = scene_manager_->GetPendingScene();
    state.active_cycle_context = scene_manager_->GetActiveCycleContext();
  }
  state.jamming_sensitivity_profile = jamming_sensitivity_profile_;
  return state;
}

void EnvironmentService::RestoreRuntimeState(
    const environment::EnvironmentServiceRuntimeState& state) {
  if (scene_manager_ != nullptr) {
    scene_manager_->RestoreState(state.active_scene_state, state.pending_scene_state,
                                 state.active_cycle_context);
  }
  current_cycle_context_ = state.active_cycle_context;
  jamming_sensitivity_profile_ = state.jamming_sensitivity_profile;
  effective_jamming_detection_threshold_db_ =
      ResolveJammingDetectionThresholdDb(jamming_sensitivity_profile_);
  RefreshFrozenSnapshotFromActiveScene();
}

void EnvironmentService::RefreshFrozenSnapshotFromActiveScene() {
  frozen_snapshot_ = EnvironmentSnapshot();
  if (scene_manager_ == nullptr || propagation_model_ == nullptr) {
    return;
  }

  const PropagationResult propagation_result =
      propagation_model_->Evaluate(scene_manager_->GetActiveScene());
  const EnvironmentSceneState& active_scene = scene_manager_->GetActiveScene();
  frozen_snapshot_.cycle_index = current_cycle_context_.cycle_index;
  frozen_snapshot_.cycle_dt_sec = current_cycle_context_.dt_sec;
  frozen_snapshot_.propagation_loss_db = propagation_result.propagation_loss_db;
  frozen_snapshot_.atmospheric_physics_loss_db = propagation_result.atmospheric_physics_loss_db;
  frozen_snapshot_.clutter_power_db = propagation_result.clutter_power_db;
  frozen_snapshot_.atmospheric_physics = active_scene.atmospheric_physics;
  frozen_snapshot_.atmospheric_context = active_scene.atmospheric_context;
  frozen_snapshot_.effective_k_factor = oneq::environment::ResolveEffectiveKFactor(
      active_scene.atmospheric_physics);
  frozen_snapshot_.effective_day_of_year = oneq::environment::ResolveEffectiveDayOfYear(
      active_scene.atmospheric_context);
  frozen_snapshot_.jammer_sources.clear();
  frozen_snapshot_.jammer_sources.reserve(active_scene.jammer_emitters.size());
  for (std::size_t i = 0; i < active_scene.jammer_emitters.size(); ++i) {
    frozen_snapshot_.jammer_sources.push_back(ToJammerSourceFact(active_scene.jammer_emitters[i]));
  }

  frozen_snapshot_.jamming_detected =
      std::find_if(frozen_snapshot_.jammer_sources.begin(), frozen_snapshot_.jammer_sources.end(),
                   [this](const JammerSourceFact& source) {
                     return source.power_db >= effective_jamming_detection_threshold_db_;
                   }) != frozen_snapshot_.jammer_sources.end();
}

}  // namespace environment
}  // namespace airborne_radar
