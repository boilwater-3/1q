// Copyright 2026. All Rights Reserved.
//
// @file EnvironmentService.cpp
// @brief 实现环境层场景冻结、传播聚合与干扰事实生成。

#include "airborne_radar/environment/EnvironmentService.h"

#include <algorithm>
#include <limits>
#include <utility>

#include "airborne_radar/environment/scene/SceneManager.h"
#include "airborne_radar/environment/simulation/PropagationModel.h"

namespace airborne_radar {
namespace environment {

namespace {

constexpr std::size_t kNoLegacyJammerEmitterIndex =
    static_cast<std::size_t>(-1);

/// @brief 将输入值裁剪到 [0, 1] 区间。
/// @param value 输入标量。
/// @return 裁剪后的结果。
float Clamp01(float value) {
  return std::max(0.0f, std::min(1.0f, value));
}

/// @brief 将输入值裁剪为非负数。
/// @param value 输入标量。
/// @return 不小于 0 的结果。
float ClampNonNegative(float value) {
  return std::max(0.0f, value);
}

/// @brief 判断旧版配置字段是否提供了兼容干扰提示。
/// @param config 旧版环境模型配置。
/// @return 存在任一旧版干扰提示时返回 true。
bool HasLegacyJammerHints(const EnvironmentModelConfig& config) {
  return config.jammer_power_db > 0.0f ||
         config.jammer_frequency_overlap_ratio > 0.0f ||
         config.jammer_prf_lock_risk > 0.0f || config.jammer_in_sidelobe;
}

/// @brief 规范化场景中的单个干扰源输入。
/// @param raw_source 原始干扰源输入。
/// @return 完成边界裁剪后的干扰源状态。
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

/// @brief 将场景干扰源转换为对外暴露的干扰事实。
/// @param emitter_state 场景中的干扰源状态。
/// @return 供快照输出使用的干扰事实。
JammerSourceFact ToJammerSourceFact(const JammerEmitterState& emitter_state) {
  const JammerEmitterState normalized = NormalizeEmitterState(emitter_state);
  JammerSourceFact source;
  source.technique = normalized.technique;
  source.power_db = normalized.power_db;
  source.js_db = normalized.js_db;
  source.frequency_overlap_ratio = normalized.frequency_overlap_ratio;
  source.prf_lock_risk = normalized.prf_lock_risk;
  source.azimuth_deg = normalized.azimuth_deg;
  source.elevation_deg = normalized.elevation_deg;
  source.angular_span_deg = normalized.angular_span_deg;
  source.in_sidelobe = normalized.in_sidelobe;
  source.confidence = normalized.confidence;
  return source;
}

/// @brief 从多源干扰事实中选取主干扰源。
/// @param sources 单周期干扰源事实列表。
/// @return 功率最大的干扰源指针；若为空则返回 nullptr。
const JammerSourceFact* SelectPrimaryJammerSource(
    const JammerSourceFactList& sources) {
  if (sources.empty()) {
    return nullptr;
  }
  return &(*std::max_element(
      sources.begin(), sources.end(),
      [](const JammerSourceFact& lhs, const JammerSourceFact& rhs) {
        return lhs.power_db < rhs.power_db;
      }));
}

/// @brief 将旧版平铺干扰配置转换为兼容场景干扰源。
/// @param config 旧版环境模型配置。
/// @return 对应的兼容干扰源状态。
JammerEmitterState ToLegacyEmitterState(const EnvironmentModelConfig& config) {
  JammerEmitterState emitter;
  emitter.technique = JammingTechnique::kUnknown;
  emitter.power_db = ClampNonNegative(config.jammer_power_db);
  emitter.frequency_overlap_ratio = Clamp01(config.jammer_frequency_overlap_ratio);
  emitter.prf_lock_risk = Clamp01(config.jammer_prf_lock_risk);
  emitter.in_sidelobe = config.jammer_in_sidelobe;
  emitter.confidence = HasLegacyJammerHints(config) ? 1.0f : 0.0f;
  return emitter;
}

/// @brief 将旧版环境配置适配为待生效场景状态。
/// @param config 旧版环境模型配置。
/// @param legacy_emitter_index 输出兼容干扰源在场景中的索引。
/// @return 统一后的环境场景状态。
EnvironmentSceneState BuildSceneStateFromModelConfig(
    const EnvironmentModelConfig& config, std::size_t* legacy_emitter_index) {
  EnvironmentSceneState scene_state;
  scene_state.base_propagation_loss_db = config.base_propagation_loss_db;
  scene_state.atmospheric_attenuation_db = config.atmospheric_attenuation_db;
  scene_state.terrain_reflection_db = config.terrain_reflection_db;
  scene_state.clutter_power_db = config.clutter_power_db;
  scene_state.jammer_emitters.reserve(
      config.jammer_sources.size() + (HasLegacyJammerHints(config) ? 1U : 0U));
  for (std::size_t i = 0; i < config.jammer_sources.size(); ++i) {
    JammerEmitterState emitter;
    emitter.technique = config.jammer_sources[i].technique;
    emitter.power_db = config.jammer_sources[i].power_db;
    emitter.js_db = config.jammer_sources[i].js_db;
    emitter.frequency_overlap_ratio =
        config.jammer_sources[i].frequency_overlap_ratio;
    emitter.prf_lock_risk = config.jammer_sources[i].prf_lock_risk;
    emitter.azimuth_deg = config.jammer_sources[i].azimuth_deg;
    emitter.elevation_deg = config.jammer_sources[i].elevation_deg;
    emitter.angular_span_deg = config.jammer_sources[i].angular_span_deg;
    emitter.in_sidelobe = config.jammer_sources[i].in_sidelobe;
    emitter.confidence = config.jammer_sources[i].confidence;
    scene_state.jammer_emitters.push_back(emitter);
  }

  if (legacy_emitter_index != nullptr) {
    *legacy_emitter_index = kNoLegacyJammerEmitterIndex;
  }
  if (HasLegacyJammerHints(config)) {
    if (legacy_emitter_index != nullptr) {
      *legacy_emitter_index = scene_state.jammer_emitters.size();
    }
    scene_state.jammer_emitters.push_back(ToLegacyEmitterState(config));
  }
  return scene_state;
}

} // namespace

EnvironmentService::EnvironmentService(EnvironmentModelConfig config)
    : scene_manager_(new scene::SceneManager(
          BuildSceneStateFromModelConfig(config,
                                         &pending_legacy_jammer_emitter_index_))),
      propagation_model_(new simulation::PropagationModel()) {
  active_legacy_jammer_emitter_index_ = pending_legacy_jammer_emitter_index_;
  RefreshFrozenSnapshotFromActiveScene();
}

EnvironmentService::~EnvironmentService() = default;

void EnvironmentService::BeginCycle(
    const EnvironmentCycleContext& cycle_context) {
  current_cycle_context_ = cycle_context;
  scene_manager_->CommitPendingScene(cycle_context);
  active_legacy_jammer_emitter_index_ = pending_legacy_jammer_emitter_index_;
  RefreshFrozenSnapshotFromActiveScene();
}

EnvironmentSnapshot EnvironmentService::SampleEnvironment() const {
  return frozen_snapshot_;
}

void EnvironmentService::UpdateSceneState(
    const EnvironmentSceneState& scene_state) {
  scene_manager_->UpdatePendingScene(scene_state);
  pending_legacy_jammer_emitter_index_ = kNoLegacyJammerEmitterIndex;
}

void EnvironmentService::UpdateModelConfig(EnvironmentModelConfig config) {
  scene_manager_->UpdatePendingScene(
      BuildSceneStateFromModelConfig(config,
                                     &pending_legacy_jammer_emitter_index_));
}

void EnvironmentService::SetJammerPowerDb(float jammer_power_db) {
  EnvironmentSceneState* pending_scene = scene_manager_->MutablePendingScene();
  if (pending_scene == nullptr) {
    return;
  }

  if (pending_legacy_jammer_emitter_index_ == kNoLegacyJammerEmitterIndex ||
      pending_legacy_jammer_emitter_index_ >= pending_scene->jammer_emitters.size()) {
    pending_legacy_jammer_emitter_index_ = pending_scene->jammer_emitters.size();
    pending_scene->jammer_emitters.push_back(JammerEmitterState());
    pending_scene->jammer_emitters.back().technique = JammingTechnique::kUnknown;
    pending_scene->jammer_emitters.back().confidence = 1.0f;
  }

  pending_scene->jammer_emitters[pending_legacy_jammer_emitter_index_].power_db =
      jammer_power_db;
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
  frozen_snapshot_.propagation_loss_db =
      propagation_result.propagation_loss_db;
  frozen_snapshot_.clutter_power_db = propagation_result.clutter_power_db;

  const EnvironmentSceneState& active_scene = scene_manager_->GetActiveScene();
  frozen_snapshot_.jammer_sources.clear();
  frozen_snapshot_.jammer_sources.reserve(active_scene.jammer_emitters.size());
  for (std::size_t i = 0; i < active_scene.jammer_emitters.size(); ++i) {
    frozen_snapshot_.jammer_sources.push_back(
        ToJammerSourceFact(active_scene.jammer_emitters[i]));
  }

  const JammerSourceFact* primary_source =
      SelectPrimaryJammerSource(frozen_snapshot_.jammer_sources);
  if (primary_source != nullptr) {
    frozen_snapshot_.jammer_power_db = primary_source->power_db;
    frozen_snapshot_.jammer_frequency_overlap_ratio =
        primary_source->frequency_overlap_ratio;
    frozen_snapshot_.jammer_prf_lock_risk = primary_source->prf_lock_risk;
    frozen_snapshot_.jammer_in_sidelobe = primary_source->in_sidelobe;
  }

  frozen_snapshot_.jamming_detected =
      std::find_if(frozen_snapshot_.jammer_sources.begin(),
                   frozen_snapshot_.jammer_sources.end(),
                   [this](const JammerSourceFact& source) {
                     return source.power_db >=
                            jamming_detection_threshold_db_;
                   }) != frozen_snapshot_.jammer_sources.end();
}

} // namespace environment
} // namespace airborne_radar
