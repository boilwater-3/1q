#include "electronic_surveillance_radar/environment/EsrEnvironmentService.h"

#include <cmath>
#include <cstddef>

#include "common/atmosphere/AtmospherePhysics.h"
#include "electronic_surveillance_radar/EsrSharedUtils.h"

namespace electronic_surveillance_radar {
namespace environment {

namespace {

/**
 * @brief 规范化单个干扰源输入。
 * @param[in] raw_source 原始输入。
 * @return 规范化后的干扰源。
 */
EsrJammerSource NormalizeJammerSource(const EsrJammerSource& raw_source) {
  EsrJammerSource normalized = raw_source;
  normalized.power_w = ClampNonNegative(raw_source.power_w);
  normalized.bandwidth_hz = std::max(0.0, raw_source.bandwidth_hz);
  normalized.deception_risk = Clamp01(raw_source.deception_risk);
  normalized.confidence = Clamp01(raw_source.confidence);
  normalized.technique = ResolveTechnique(normalized);
  normalized.active =
      raw_source.active && normalized.power_w > 0.0f && normalized.bandwidth_hz > 0.0;
  return normalized;
}

/**
 * @brief 根据周期上下文构造冻结快照。
 * @param[in] cycle_context 周期上下文。
 * @param[in] config 环境配置。
 * @return 冻结环境快照。
 */
EsrEnvironmentSnapshot BuildSnapshot(const EsrEnvironmentCycleContext& cycle_context,
                                     const EsrEnvironmentModelConfig& config) {
  EsrEnvironmentSnapshot snapshot;
  snapshot.cycle_index = cycle_context.cycle_index;
  snapshot.dt_sec = cycle_context.dt_sec;
  const EsrAtmosphericPhysicsConfig& atmospheric_physics =
      cycle_context.scene_state.atmospheric_physics.enable_physical_model
          ? cycle_context.scene_state.atmospheric_physics
          : config.atmospheric_physics;
  float physical_loss_db = 0.0f;
  if (atmospheric_physics.enable_physical_model) {
    oneq::internal::atmosphere::AtmosphericPropagationInputs physics_inputs;
    physics_inputs.enable_physics = true;
    physics_inputs.frequency_hz = atmospheric_physics.frequency_hz;
    physics_inputs.path_length_m = atmospheric_physics.path_length_m;
    physics_inputs.radar_altitude_m = atmospheric_physics.radar_altitude_m;
    physics_inputs.target_altitude_m = atmospheric_physics.target_altitude_m;
    physics_inputs.elevation_deg = atmospheric_physics.elevation_deg;
    physics_inputs.pressure_hpa = atmospheric_physics.pressure_hpa;
    physics_inputs.temperature_k = atmospheric_physics.temperature_k;
    physics_inputs.relative_humidity = atmospheric_physics.relative_humidity;
    physics_inputs.k_factor = atmospheric_physics.k_factor;
    physics_inputs.day_of_year = atmospheric_physics.day_of_year;
    physics_inputs.solar_flux_f107a = atmospheric_physics.solar_flux_f107a;
    physics_inputs.solar_flux_f107 = atmospheric_physics.solar_flux_f107;
    physics_inputs.geomagnetic_ap = atmospheric_physics.geomagnetic_ap;
    const oneq::internal::atmosphere::AtmosphericPropagationResult physics_result =
        oneq::internal::atmosphere::EvaluateAtmosphericPropagation(physics_inputs);
    physical_loss_db = physics_result.total_physics_loss_db;
  }
  snapshot.propagation_loss_db =
      ClampNonNegative(cycle_context.scene_state.base_propagation_loss_db +
                       cycle_context.scene_state.atmospheric_attenuation_db +
                       cycle_context.scene_state.terrain_reflection_db + physical_loss_db);

  const float clutter_noise = cycle_context.scene_state.clutter_noise_w > 0.0f
                                  ? cycle_context.scene_state.clutter_noise_w
                                  : config.default_clutter_noise_w;
  snapshot.clutter_noise_w = ClampNonNegative(clutter_noise);
  snapshot.spectrum_occupancy_ratio = Clamp01(cycle_context.scene_state.spectrum_occupancy_ratio);

  snapshot.jammer_sources.clear();
  snapshot.jammer_sources.reserve(cycle_context.scene_state.jammer_sources.size());
  snapshot.suppression_power_w = 0.0f;
  snapshot.jammer_power_w = 0.0f;
  snapshot.deception_risk = 0.0f;
  float deception_clear_probability = 1.0f;
  for (std::size_t i = 0; i < cycle_context.scene_state.jammer_sources.size(); ++i) {
    const EsrJammerSource source =
        NormalizeJammerSource(cycle_context.scene_state.jammer_sources[i]);
    snapshot.jammer_sources.push_back(source);
    if (!source.active) {
      continue;
    }

    const float weighted_power = source.power_w * source.confidence;
    if (HasSuppressionEffect(source.technique)) {
      snapshot.suppression_power_w += weighted_power;
    }
    if (HasDeceptionEffect(source.technique)) {
      const float source_risk = Clamp01(source.deception_risk * source.confidence);
      const float safe_source_risk = std::isfinite(source_risk) ? source_risk : 0.0f;
      deception_clear_probability *= (1.0f - safe_source_risk);
      deception_clear_probability = Clamp01(deception_clear_probability);
    }
  }
  snapshot.deception_risk = Clamp01(1.0f - deception_clear_probability);
  snapshot.jammer_power_w = snapshot.suppression_power_w;

  snapshot.jamming_detected = snapshot.suppression_power_w >= config.jamming_detection_threshold_w;
  return snapshot;
}

}  // namespace

EsrEnvironmentService::EsrEnvironmentService(EsrEnvironmentModelConfig config) : config_(config) {}

void EsrEnvironmentService::BeginCycle(const EsrEnvironmentCycleContext& cycle_context) {
  frozen_snapshot_ = BuildSnapshot(cycle_context, config_);
}

EsrEnvironmentSnapshot EsrEnvironmentService::SampleEnvironment() const { return frozen_snapshot_; }

void EsrEnvironmentService::UpdateModelConfig(EsrEnvironmentModelConfig config) {
  config_ = config;
}

}  // namespace environment
}  // namespace electronic_surveillance_radar
