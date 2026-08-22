#include "electronic_surveillance_radar/environment/EsrEnvironmentService.h"

#include <cmath>
#include <cstddef>

#include "1q/environment/PropagationPhysics.h"
#include "common/numerics/ClampUtils.h"

namespace electronic_surveillance_radar {
namespace environment {

using config::EsrAtmosphericObservation;
using config::EsrAtmosphericPhysicsConfig;
using config::EsrClutterDensityLevel;
using config::EsrEnvironmentConfig;
using config::EsrEnvironmentScenarioConfig;
using config::EsrPropagationEnvironmentProfile;
using session::EsrEnvironmentSnapshot;

namespace {

constexpr float kDefaultAtmosphereFrequencyHz = 10.0e9f;
constexpr float kDefaultAtmospherePathLengthM = 10.0e3f;
constexpr float kDefaultAtmosphereElevationDeg = 5.0f;

float ResolvePropagationProfileLossDb(config::EsrPropagationEnvironmentProfile profile) {
  switch (profile) {
    case config::EsrPropagationEnvironmentProfile::kOpen:
      return 2.0f;
    case config::EsrPropagationEnvironmentProfile::kComplex:
      return 7.0f;
    case config::EsrPropagationEnvironmentProfile::kTypical:
    default:
      return 4.0f;
  }
}

float ResolveWeatherLossDb(const config::EsrAtmosphericPhysicsConfig& physics,
                           const config::EsrAtmosphericObservation& observation) {
  const float humidity = oneq::common::numerics::Clamp01(physics.relative_humidity);
  const float precipitation =
      oneq::common::numerics::ClampNonNegative(observation.precipitation_rate_mmph);
  const float visibility_km = std::max(0.5f, observation.visibility_km);
  const float humidity_loss_db = 1.5f * humidity;
  const float precipitation_loss_db = 0.12f * precipitation;
  const float visibility_loss_db = visibility_km < 20.0f ? (20.0f - visibility_km) * 0.08f : 0.0f;
  return oneq::common::numerics::ClampNonNegative(humidity_loss_db + precipitation_loss_db +
                                                  visibility_loss_db);
}

float ResolveClutterNoiseW(const config::EsrEnvironmentScenarioConfig& config) {
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
  switch (config.clutter_density) {
    case config::EsrClutterDensityLevel::kLow:
      return reference_noise * 0.6f;
    case config::EsrClutterDensityLevel::kHigh:
      return reference_noise * 2.0f;
    case config::EsrClutterDensityLevel::kMedium:
    default:
      return reference_noise;
  }
}

/**
 * @brief 根据周期上下文和配置构造冻结快照。
 * @param[in] cycle_index 当前周期号。
 * @param[in] dt_sec 当前周期步长。
 * @param[in] platform_altitude_m 接收平台海拔（单位：m）。
 * @param[in] config 环境模型配置。
 * @return 冻结环境快照。
 */
session::EsrEnvironmentSnapshot BuildSnapshot(std::uint32_t cycle_index, float dt_sec,
                                               float platform_altitude_m,
                                               const config::EsrEnvironmentScenarioConfig& config) {
  session::EsrEnvironmentSnapshot snapshot;
  snapshot.cycle_index = cycle_index;
  snapshot.dt_sec = dt_sec;
  const config::EsrAtmosphericPhysicsConfig& atmospheric_physics = config.atmospheric_physics;
  float physical_loss_db = 0.0f;
  if (atmospheric_physics.enable_physical_model) {
    oneq::environment::AtmosphericObservation obs;
    obs.pressure_hpa = atmospheric_physics.pressure_hpa;
    obs.temperature_k = atmospheric_physics.temperature_k;
    obs.relative_humidity = atmospheric_physics.relative_humidity;
    oneq::environment::PropagationInputs inputs = oneq::environment::BuildPropagationInputs(
        kDefaultAtmosphereFrequencyHz, kDefaultAtmospherePathLengthM,
        std::max(0.0f, platform_altitude_m),
        std::max(0.0f, platform_altitude_m), kDefaultAtmosphereElevationDeg, obs);
    physical_loss_db = oneq::environment::EvaluatePropagation(inputs).total_physics_loss_db;
  }
  const float semantic_loss_db =
      ResolvePropagationProfileLossDb(config.propagation_profile) +
      ResolveWeatherLossDb(config.atmospheric_physics, config.atmospheric_observation);
  snapshot.propagation_loss_db =
      oneq::common::numerics::ClampNonNegative(semantic_loss_db + physical_loss_db);
  snapshot.clutter_noise_w = ResolveClutterNoiseW(config);
  snapshot.spectrum_occupancy_ratio =
      oneq::common::numerics::Clamp01(config.spectrum_occupancy_ratio);
  return snapshot;
}

}  // namespace

EsrEnvironmentService::EsrEnvironmentService(config::EsrEnvironmentScenarioConfig config)
    : config_(config) {}

void EsrEnvironmentService::BeginCycle(std::uint32_t cycle_index, float dt_sec,
                                        float platform_altitude_m) {
  frozen_snapshot_ = BuildSnapshot(cycle_index, dt_sec, platform_altitude_m, config_);
}

session::EsrEnvironmentSnapshot EsrEnvironmentService::SampleEnvironment() const {
  return frozen_snapshot_;
}

void EsrEnvironmentService::UpdateModelConfig(config::EsrEnvironmentScenarioConfig config) {
  config_ = config;
}

}  // namespace environment
}  // namespace electronic_surveillance_radar
