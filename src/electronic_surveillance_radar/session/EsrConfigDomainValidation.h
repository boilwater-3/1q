/**
 * @file EsrConfigDomainValidation.h
 * @brief ESR 静态配置、运行期补丁与 replay 解码共享的领域不变量。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CONFIG_DOMAIN_VALIDATION_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CONFIG_DOMAIN_VALIDATION_H_

#include <cstdint>

#include "1q/electronic_surveillance_radar/config/EsrEnvironmentConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrMissionConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrPolicyConfig.h"
#include "common/validation/ValidationUtils.h"

namespace electronic_surveillance_radar {
namespace session {
namespace config_validation {

inline bool IsValidWorkMode(config::EsrWorkMode value) {
  const std::int32_t raw = static_cast<std::int32_t>(value);
  return raw >= static_cast<std::int32_t>(config::EsrWorkMode::kEsm) &&
         raw <= static_cast<std::int32_t>(config::EsrWorkMode::kRwr);
}

inline bool IsValidScanStartPosition(config::EsrScanStartPosition value) {
  const std::int32_t raw = static_cast<std::int32_t>(value);
  return raw >=
             static_cast<std::int32_t>(config::EsrScanStartPosition::kLeftTop) &&
         raw <= static_cast<std::int32_t>(
                    config::EsrScanStartPosition::kLeftBottom);
}

inline bool IsValidScanSequence(config::EsrScanSequence value) {
  const std::int32_t raw = static_cast<std::int32_t>(value);
  return raw >=
             static_cast<std::int32_t>(config::EsrScanSequence::kAzimuthFirst) &&
         raw <= static_cast<std::int32_t>(
                    config::EsrScanSequence::kElevationFirst);
}

inline bool IsValidEnvironmentPreset(config::EsrEnvironmentPreset value) {
  const std::int32_t raw = static_cast<std::int32_t>(value);
  return raw >=
             static_cast<std::int32_t>(config::EsrEnvironmentPreset::kStandard) &&
         raw <=
             static_cast<std::int32_t>(config::EsrEnvironmentPreset::kJammed);
}

inline bool IsValidDetectionPolicy(
    const config::EsrDetectionPolicyConfig& detection) {
  return oneq::common::validation::IsFinite(detection.minimum_snr_db) &&
         oneq::common::validation::IsFinite(detection.pfa) &&
         detection.pfa > 0.0f && detection.pfa < 1.0f &&
         detection.pulse_count > 0U &&
         oneq::common::validation::IsFinite(detection.threshold_scale) &&
         detection.threshold_scale > 0.0f;
}

inline bool IsValidAtmosphericPhysics(
    const config::EsrAtmosphericPhysicsConfig& atmosphere) {
  if (!atmosphere.enable_physical_model) {
    return true;
  }
  return oneq::common::validation::IsFinite(atmosphere.pressure_hpa) &&
         atmosphere.pressure_hpa > 0.0f &&
         oneq::common::validation::IsFinite(atmosphere.temperature_k) &&
         atmosphere.temperature_k > 0.0f &&
         oneq::common::validation::IsFinite(atmosphere.relative_humidity) &&
         atmosphere.relative_humidity >= 0.0f &&
         atmosphere.relative_humidity <= 1.0f;
}

inline bool IsValidEnvironment(
    const config::EsrEnvironmentScenarioConfig& environment) {
  return IsValidEnvironmentPreset(environment.preset) &&
         IsValidAtmosphericPhysics(environment.atmospheric_physics) &&
         oneq::common::validation::IsRatio01(environment.spectrum_occupancy_ratio) &&
         oneq::common::validation::IsFinite(
             environment.atmospheric_observation.precipitation_rate_mmph) &&
         environment.atmospheric_observation.precipitation_rate_mmph >= 0.0f &&
         oneq::common::validation::IsFinite(environment.atmospheric_observation.visibility_km) &&
         environment.atmospheric_observation.visibility_km > 0.0f;
}

inline bool IsValidMissionEnums(const config::EsrMissionConfig& mission) {
  return IsValidWorkMode(mission.work_mode) &&
         IsValidScanStartPosition(mission.scan.scan_start_position) &&
         IsValidScanSequence(mission.scan.scan_sequence);
}

}  // namespace config_validation
}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CONFIG_DOMAIN_VALIDATION_H_
