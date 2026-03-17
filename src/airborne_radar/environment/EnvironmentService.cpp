// Copyright 2026. All Rights Reserved.
//
// Description: EnvironmentService 的实现。

#include "1q/airborne_radar/environment/EnvironmentService.h"

#include <algorithm>

namespace airborne_radar {
namespace environment {

EnvironmentService::EnvironmentService(EnvironmentModelConfig config)
    : config_(config) {}

EnvironmentSnapshot EnvironmentService::SampleEnvironment() const {
  EnvironmentSnapshot snapshot{};

  snapshot.propagation_loss_db =
      std::max(0.0f, config_.base_propagation_loss_db +
                         config_.atmospheric_attenuation_db +
                         config_.terrain_reflection_db);
  snapshot.clutter_power_db = std::max(0.0f, config_.clutter_power_db);
  snapshot.jammer_power_db = std::max(0.0f, config_.jammer_power_db);
  snapshot.jammer_frequency_overlap_ratio =
      std::max(0.0f, std::min(1.0f, config_.jammer_frequency_overlap_ratio));
  snapshot.jammer_prf_lock_risk =
      std::max(0.0f, std::min(1.0f, config_.jammer_prf_lock_risk));
  snapshot.jammer_in_sidelobe = config_.jammer_in_sidelobe;
  snapshot.jamming_detected =
      config_.jammer_power_db >= jamming_detection_threshold_db_;

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
