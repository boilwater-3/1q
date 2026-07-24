#include "electronic_surveillance_radar/environment/EsrEnvironmentService.h"

#include <cmath>
#include <cstddef>

#include "1q/environment/PropagationPhysics.h"
#include "electronic_surveillance_radar/environment/EsrSharedUtils.h"

namespace electronic_surveillance_radar {
namespace environment {

using config::EsrAtmosphericPhysicsConfig;
using config::EsrEnvironmentConfig;
using config::EsrEnvironmentScenarioConfig;
using session::EsrAtmosphericObservation;
using session::EsrClutterDensityLevel;
using session::EsrEnvironmentCycleContext;
using session::EsrEnvironmentInput;
using session::EsrEnvironmentSnapshot;
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
    physical_loss_db = oneq::environment::EvaluatePropagation(inputs).total_physics_loss_db;
  }
  const float semantic_loss_db = ResolvePropagationProfileLossDb(observation.propagation_profile) +
                                 ResolveWeatherLossDb(observation.atmospheric_observation);
  snapshot.propagation_loss_db = utils::ClampNonNegative(semantic_loss_db + physical_loss_db);
  snapshot.clutter_noise_w = ResolveClutterNoiseW(observation, config);
  snapshot.spectrum_occupancy_ratio = utils::Clamp01(observation.spectrum_occupancy_ratio);
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
