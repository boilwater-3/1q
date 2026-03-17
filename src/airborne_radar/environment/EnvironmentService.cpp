// Copyright 2026. All Rights Reserved.
//
// Description: EnvironmentService 的实现。

#include "1q/airborne_radar/environment/EnvironmentService.h"

#include <algorithm>

namespace airborne_radar {
namespace environment {

namespace {

float Clamp01(float value) {
  return std::max(0.0f, std::min(1.0f, value));
}

float ClampNonNegative(float value) {
  return std::max(0.0f, value);
}

bool HasLegacyJammerHints(const EnvironmentModelConfig& config) {
  return config.jammer_power_db > 0.0f ||
         config.jammer_frequency_overlap_ratio > 0.0f ||
         config.jammer_prf_lock_risk > 0.0f || config.jammer_in_sidelobe;
}

JammerSourceFact BuildLegacyJammerSource(const EnvironmentModelConfig& config) {
  JammerSourceFact source;
  source.technique = JammingTechnique::kUnknown;
  source.power_db = ClampNonNegative(config.jammer_power_db);
  source.frequency_overlap_ratio = Clamp01(config.jammer_frequency_overlap_ratio);
  source.prf_lock_risk = Clamp01(config.jammer_prf_lock_risk);
  source.in_sidelobe = config.jammer_in_sidelobe;
  source.confidence = HasLegacyJammerHints(config) ? 1.0f : 0.0f;
  return source;
}

JammerSourceFact NormalizeJammerSource(const JammerSourceFact& raw_source) {
  JammerSourceFact normalized = raw_source;
  normalized.power_db = ClampNonNegative(raw_source.power_db);
  normalized.js_db = ClampNonNegative(raw_source.js_db);
  normalized.frequency_overlap_ratio = Clamp01(raw_source.frequency_overlap_ratio);
  normalized.prf_lock_risk = Clamp01(raw_source.prf_lock_risk);
  normalized.angular_span_deg = ClampNonNegative(raw_source.angular_span_deg);
  normalized.confidence = Clamp01(raw_source.confidence);
  return normalized;
}

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

} // namespace

EnvironmentService::EnvironmentService(EnvironmentModelConfig config)
    : config_(config) {}

EnvironmentSnapshot EnvironmentService::SampleEnvironment() const {
  EnvironmentSnapshot snapshot{};

  snapshot.propagation_loss_db =
      std::max(0.0f, config_.base_propagation_loss_db +
                         config_.atmospheric_attenuation_db +
                         config_.terrain_reflection_db);
  snapshot.clutter_power_db = std::max(0.0f, config_.clutter_power_db);

  snapshot.jammer_sources.clear();
  snapshot.jammer_sources.reserve(
      config_.jammer_sources.size() + (HasLegacyJammerHints(config_) ? 1U : 0U));
  for (std::size_t i = 0; i < config_.jammer_sources.size(); ++i) {
    snapshot.jammer_sources.push_back(NormalizeJammerSource(config_.jammer_sources[i]));
  }
  if (HasLegacyJammerHints(config_)) {
    snapshot.jammer_sources.push_back(BuildLegacyJammerSource(config_));
  }

  const JammerSourceFact* primary_source =
      SelectPrimaryJammerSource(snapshot.jammer_sources);
  if (primary_source != nullptr) {
    snapshot.jammer_power_db = primary_source->power_db;
    snapshot.jammer_frequency_overlap_ratio =
        primary_source->frequency_overlap_ratio;
    snapshot.jammer_prf_lock_risk = primary_source->prf_lock_risk;
    snapshot.jammer_in_sidelobe = primary_source->in_sidelobe;
  }
  snapshot.jamming_detected = std::find_if(
                                  snapshot.jammer_sources.begin(),
                                  snapshot.jammer_sources.end(),
                                  [this](const JammerSourceFact& source) {
                                    return source.power_db >=
                                           jamming_detection_threshold_db_;
                                  }) != snapshot.jammer_sources.end();

  return snapshot;
}

void EnvironmentService::UpdateModelConfig(EnvironmentModelConfig config) {
  config_ = config;
}

void EnvironmentService::SetJammerPowerDb(float jammer_power_db) {
  config_.jammer_power_db = jammer_power_db;
}

void EnvironmentService::SetJammingDetectionThresholdDb(float threshold_db) {
  jamming_detection_threshold_db_ = threshold_db;
}

} // namespace environment
} // namespace airborne_radar
