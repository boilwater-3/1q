#include "electronic_surveillance_radar/environment/EsrEnvironmentService.h"

#include <cmath>
#include <cstddef>

#include "1q/environment/PropagationPhysics.h"
#include "electronic_surveillance_radar/environment/EsrSharedUtils.h"

namespace electronic_surveillance_radar {
namespace environment {

using config::EsrAtmosphericDerivedContext;
using config::EsrAtmosphericPhysicsConfig;
using config::EsrEnvironmentConfig;
using config::EsrEnvironmentScenarioConfig;
using session::EsrAtmosphericObservation;
using session::EsrClutterDensityLevel;
using session::EsrEnvironmentCycleContext;
using session::EsrEnvironmentInput;
using session::EsrEnvironmentSnapshot;
using session::EsrJammerSource;
using session::EsrJammingTechnique;
using session::EsrPropagationEnvironmentProfile;

namespace {

constexpr float kDefaultAtmosphereFrequencyHz = 10.0e9f;
constexpr float kDefaultAtmospherePathLengthM = 10.0e3f;
constexpr float kDefaultAtmosphereElevationDeg = 5.0f;

float ResolvePropagationProfileLossDb(session::EsrPropagationEnvironmentProfile profile) {
  switch (profile) {
    case session::EsrPropagationEnvironmentProfile::kOpen:
      return 2.0f;
    case session::EsrPropagationEnvironmentProfile::kComplex:
      return 7.0f;
    case session::EsrPropagationEnvironmentProfile::kTypical:
    default:
      return 4.0f;
  }
}

float ResolveWeatherLossDb(const session::EsrAtmosphericObservation& observation) {
  const float humidity = utils::Clamp01(observation.relative_humidity_ratio);
  const float precipitation = utils::ClampNonNegative(observation.precipitation_rate_mmph);
  const float visibility_km = std::max(0.5f, observation.visibility_km);
  const float humidity_loss_db = 1.5f * humidity;
  const float precipitation_loss_db = 0.12f * precipitation;
  const float visibility_loss_db = visibility_km < 20.0f ? (20.0f - visibility_km) * 0.08f : 0.0f;
  return utils::ClampNonNegative(humidity_loss_db + precipitation_loss_db + visibility_loss_db);
}

float ResolveClutterNoiseW(const session::EsrEnvironmentInput& observation,
                           const config::EsrEnvironmentScenarioConfig& config) {
  float reference_noise = 1.0e-12f;
  switch (config.preset) {
    case config::EsrEnvironmentPreset::kLowClutter:
      reference_noise = 5.0e-13f;
      break;
    case config::EsrEnvironmentPreset::kDenseClutter:
    case config::EsrEnvironmentPreset::kJammed:
      reference_noise = 5.0e-12f;
      break;
    case config::EsrEnvironmentPreset::kStandard:
    default:
      reference_noise = 1.0e-12f;
      break;
  }
  switch (observation.clutter_density) {
    case session::EsrClutterDensityLevel::kLow:
      return reference_noise * 0.6f;
    case session::EsrClutterDensityLevel::kHigh:
      return reference_noise * 2.0f;
    case session::EsrClutterDensityLevel::kMedium:
    default:
      return reference_noise;
  }
}

float ResolveJammingDetectionThresholdW(const config::EsrEnvironmentScenarioConfig& config) {
  switch (config.preset) {
    case config::EsrEnvironmentPreset::kLowClutter:
      return 8.0e-10f;
    case config::EsrEnvironmentPreset::kJammed:
      return 5.0e-9f;
    case config::EsrEnvironmentPreset::kDenseClutter:
    case config::EsrEnvironmentPreset::kStandard:
    default:
      return 2.0e-9f;
  }
}

/**
 * @brief 规范化单个干扰源输入。
 * @param[in] raw_source 原始输入。
 * @return 规范化后的干扰源。
 */
EsrJammerSource NormalizeJammerSource(const EsrJammerSource& raw_source) {
  EsrJammerSource normalized = raw_source;
  normalized.power_w = utils::ClampNonNegative(raw_source.power_w);
  normalized.bandwidth_hz = std::max(0.0, raw_source.bandwidth_hz);
  normalized.deception_risk = utils::Clamp01(raw_source.deception_risk);
  normalized.confidence = utils::Clamp01(raw_source.confidence);
  normalized.technique = utils::ResolveTechnique(normalized);
  normalized.active =
      raw_source.active && normalized.power_w > 0.0f && normalized.bandwidth_hz > 0.0;
  return normalized;
}

/**
 * @brief 根据周期上下文构造冻结快照。
 * @param[in] cycle_context 周期上下文。
 * @param[in] config 环境模型配置。
 * @return 冻结环境快照。
 */
session::EsrEnvironmentSnapshot BuildSnapshot(
    const session::EsrEnvironmentCycleContext& cycle_context,
    const config::EsrEnvironmentScenarioConfig& config) {
  session::EsrEnvironmentSnapshot snapshot;
  snapshot.cycle_index = cycle_context.cycle_index;
  snapshot.dt_sec = cycle_context.dt_sec;
  const session::EsrEnvironmentInput& observation = cycle_context.observation;
  const config::EsrAtmosphericPhysicsConfig& atmospheric_physics = config.atmospheric_physics;
  const config::EsrAtmosphericDerivedContext& atmospheric_context = config.atmospheric_context;
  float physical_loss_db = 0.0f;
  if (atmospheric_physics.enable_physical_model) {
    oneq::environment::AtmosphericObservation obs;
    obs.pressure_hpa = atmospheric_physics.pressure_hpa;
    obs.temperature_k = atmospheric_physics.temperature_k;
    obs.relative_humidity = atmospheric_physics.relative_humidity;
    oneq::environment::PropagationInputs inputs = oneq::environment::BuildPropagationInputs(
        kDefaultAtmosphereFrequencyHz, kDefaultAtmospherePathLengthM,
        std::max(0.0f, cycle_context.platform_altitude_m),
        std::max(0.0f, cycle_context.platform_altitude_m), kDefaultAtmosphereElevationDeg, obs);
    inputs.has_space_weather_context = true;
    inputs.space_weather_context = atmospheric_context;
    physical_loss_db = oneq::environment::EvaluatePropagation(inputs).total_physics_loss_db;
  }
  const float semantic_loss_db = ResolvePropagationProfileLossDb(observation.propagation_profile) +
                                 ResolveWeatherLossDb(observation.atmospheric_observation);
  snapshot.propagation_loss_db = utils::ClampNonNegative(semantic_loss_db + physical_loss_db);
  snapshot.clutter_noise_w = ResolveClutterNoiseW(observation, config);
  snapshot.spectrum_occupancy_ratio = utils::Clamp01(observation.spectrum_occupancy_ratio);
  snapshot.interference_mode = observation.interference_mode;
  snapshot.engineering_emissions = observation.engineering_emissions;

  snapshot.jammer_sources.clear();
  snapshot.jammer_sources.reserve(observation.jammer_sources.size());
  snapshot.suppression_power_w = 0.0f;
  snapshot.deception_risk = 0.0f;
  float deception_clear_probability = 1.0f;
  for (std::size_t i = 0; i < observation.jammer_sources.size(); ++i) {
    const EsrJammerSource source = NormalizeJammerSource(observation.jammer_sources[i]);
    snapshot.jammer_sources.push_back(source);
    if (!source.active) {
      continue;
    }

    const float weighted_power = source.power_w * source.confidence;
    if (utils::HasSuppressionEffect(source.technique)) {
      snapshot.suppression_power_w += weighted_power;
    }
    if (utils::HasDeceptionEffect(source.technique)) {
      const float source_risk = utils::Clamp01(source.deception_risk * source.confidence);
      const float safe_source_risk = std::isfinite(source_risk) ? source_risk : 0.0f;
      deception_clear_probability *= (1.0f - safe_source_risk);
      deception_clear_probability = utils::Clamp01(deception_clear_probability);
    }
  }
  snapshot.deception_risk = utils::Clamp01(1.0f - deception_clear_probability);

  snapshot.jamming_detected =
      snapshot.suppression_power_w >= ResolveJammingDetectionThresholdW(config);
  return snapshot;
}

}  // namespace

EsrEnvironmentService::EsrEnvironmentService(config::EsrEnvironmentScenarioConfig config)
    : config_(config) {}

void EsrEnvironmentService::BeginCycle(const session::EsrEnvironmentCycleContext& cycle_context) {
  frozen_snapshot_ = BuildSnapshot(cycle_context, config_);
}

session::EsrEnvironmentSnapshot EsrEnvironmentService::SampleEnvironment() const {
  return frozen_snapshot_;
}

void EsrEnvironmentService::UpdateModelConfig(config::EsrEnvironmentScenarioConfig config) {
  config_ = config;
}

}  // namespace environment
}  // namespace electronic_surveillance_radar
