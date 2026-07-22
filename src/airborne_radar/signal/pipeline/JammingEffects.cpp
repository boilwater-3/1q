#include "airborne_radar/signal/pipeline/JammingEffects.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "airborne_radar/signal/detection/BeamwidthResolution.h"
#include "airborne_radar/utils/MathUtils.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

using JammingEffectsConfig = ::airborne_radar::config::execution::JammingEffectsConfig;

namespace {

constexpr float kSidelobeCancellerResidual = 0.55f;
constexpr float kAdaptiveBeamNarrowResidual = 0.72f;
constexpr float kAdaptiveBeamWideResidual = 0.82f;
constexpr float kAgilityFreqResidualMin = 0.35f;
constexpr float kEccmRejitterResidualMin = 0.30f;
constexpr float kBurnthroughResidualMin = 0.55f;
constexpr float kResidualFactorMin = 0.15f;
constexpr float kNoiseSidelobeCancellerFactor = 0.75f;
constexpr float kDeceptionAgilityFactor = 0.65f;
constexpr float kDeceptionRejitterFactor = 0.65f;
constexpr float kRepeaterRejitterFactor = 0.60f;

constexpr float kMixedSemanticDominanceRatio = 0.65f;
constexpr float kMixedSemanticMinScore = 0.18f;
constexpr float kUnknownNoiseSplitWeight = 0.5f;

constexpr float kPhysicalWeightNoise = 1.0f;
constexpr float kPhysicalWeightDeception = 0.20f;
constexpr float kPhysicalWeightRepeater = 0.40f;
constexpr float kPhysicalWeightUnknown = 0.70f;
constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;

float ResolveJammerConfidenceWeight(const JammingEffectsConfig& cfg,
                                    const session::JammerSourceFact& jammer_source) {
  return utils::ClampFloat(jammer_source.confidence, cfg.confidence_weight_min, 1.0f);
}

bool TryBuildReceiverSite(const ExecutionConfig& config,
                          const session::EnvironmentSnapshot& environment_snapshot,
                          oneq::electromagnetics::RfReceiverSite* receiver) {
  if (receiver == nullptr || !environment_snapshot.has_rf_receiver_kinematics) {
    return false;
  }
  oneq::coordinate::LlaPositionDegM receiver_lla;
  if (!oneq::coordinate::TryEcefToLla(environment_snapshot.rf_receiver_position_ecef_m,
                                      &receiver_lla)) {
    return false;
  }

  const double azimuth_rad =
      static_cast<double>(config.detection.orientation.scan_center_deg.az_deg) * kDegreesToRadians;
  const double elevation_rad =
      static_cast<double>(config.detection.orientation.scan_center_deg.el_deg) * kDegreesToRadians;
  const double cos_elevation = std::cos(elevation_rad);
  oneq::coordinate::Vector3d local_direction;
  local_direction.x = std::sin(azimuth_rad) * cos_elevation;
  local_direction.y = std::cos(azimuth_rad) * cos_elevation;
  local_direction.z = std::sin(elevation_rad);
  oneq::coordinate::EulerAnglesDeg attitude;
  attitude.yaw_deg = config.detection.platform_attitude_deg.yaw_deg;
  attitude.pitch_deg = config.detection.platform_attitude_deg.pitch_deg;
  attitude.roll_deg = config.detection.platform_attitude_deg.roll_deg;
  const oneq::coordinate::Vector3d enu_direction = oneq::coordinate::RotateLocalToEnu(
      local_direction.x, local_direction.y, local_direction.z, attitude);
  oneq::coordinate::Vector3d ecef_direction;
  if (!oneq::coordinate::TryEnuToEcefDirection(enu_direction, receiver_lla, &ecef_direction)) {
    return false;
  }

  const config::engineering::DetectionConfig& detection_config = config.detection.engineering;
  const config::engineering::ReceiverConfig& receiver_config = detection_config.receiver;
  const float frequency_hz = detection_config.transmitter.frequency_hz;
  const float wavelength_m = frequency_hz > 0.0f ? 299792458.0f / frequency_hz : 0.0f;
  const detection::EffectiveBeamwidthDeg beamwidth = detection::ResolveEffectiveBeamwidth(
      detection_config.antenna, config.detection.orientation, wavelength_m);

  receiver->entity_id = environment_snapshot.rf_receiver_entity_id;
  receiver->position_ecef_m = environment_snapshot.rf_receiver_position_ecef_m;
  receiver->velocity_ecef_mps = environment_snapshot.rf_receiver_velocity_ecef_mps;
  receiver->polarization = receiver_config.polarization;
  receiver->window_start_time_s = 0.0;
  receiver->window_duration_s = static_cast<double>(environment_snapshot.cycle_dt_sec);
  receiver->center_frequency_hz = static_cast<double>(frequency_hz);
  receiver->bandwidth_hz = static_cast<double>(detection_config.transmitter.bandwidth_hz);
  receiver->receiver_system_loss_db = static_cast<double>(receiver_config.receive_loss_db);
  receiver->minimum_far_field_range_m =
      static_cast<double>(receiver_config.minimum_far_field_range_m);
  receiver->has_co_site_isolation = receiver_config.has_co_site_isolation;
  receiver->co_site_isolation_db = static_cast<double>(receiver_config.co_site_isolation_db);
  receiver->antenna.boresight_ecef_unit.x = ecef_direction.x;
  receiver->antenna.boresight_ecef_unit.y = ecef_direction.y;
  receiver->antenna.boresight_ecef_unit.z = ecef_direction.z;
  receiver->antenna.peak_gain_dbi = static_cast<double>(detection_config.antenna.main_beam_gain_db);
  receiver->antenna.half_power_beamwidth_deg =
      static_cast<double>(std::max(beamwidth.az_beamwidth_deg, beamwidth.el_beamwidth_deg));
  receiver->antenna.sidelobe_level_db =
      static_cast<double>(detection_config.antenna.pattern.max_sidelobe_level_db);
  receiver->antenna.backlobe_level_db =
      static_cast<double>(detection_config.antenna.pattern.backlobe_level_db);
  receiver->antenna.cross_polarization_isolation_db =
      static_cast<double>(receiver_config.cross_polarization_isolation_db);
  return true;
}

float ComputeTrackLevelJammingContribution(const session::ArControlProfile& control_profile,
                                           const session::JammerSourceFact& jammer_source) {
  const float confidence_weight = utils::ClampFloat(jammer_source.confidence, 0.25f, 1.0f);
  const float residual_factor = ComputeResidualJammerFactor(control_profile, jammer_source);
  float contribution = 0.10f + 0.020f * utils::ClampFloat(jammer_source.power_db, 0.0f, 20.0f) +
                       0.015f * utils::ClampFloat(jammer_source.js_db, 0.0f, 12.0f);

  switch (jammer_source.technique) {
    case config::JammingTechnique::kNoiseSuppression:
      contribution += jammer_source.in_sidelobe ? 0.14f : 0.08f;
      break;
    case config::JammingTechnique::kDeception:
      contribution += 0.25f * utils::ClampFloat(jammer_source.frequency_overlap_ratio, 0.0f, 1.0f);
      contribution += 0.22f * utils::ClampFloat(jammer_source.prf_lock_risk, 0.0f, 1.0f);
      break;
    case config::JammingTechnique::kRepeater:
      contribution += 0.18f * utils::ClampFloat(jammer_source.prf_lock_risk, 0.0f, 1.0f);
      contribution += 0.14f * utils::ClampFloat(jammer_source.frequency_overlap_ratio, 0.0f, 1.0f);
      break;
    case config::JammingTechnique::kUnknown:
    default:
      contribution += 0.15f * utils::ClampFloat(jammer_source.frequency_overlap_ratio, 0.0f, 1.0f);
      contribution += 0.12f * utils::ClampFloat(jammer_source.prf_lock_risk, 0.0f, 1.0f);
      break;
  }

  return utils::ClampFloat(contribution * confidence_weight * residual_factor, 0.0f, 1.0f);
}

}  // namespace

bool HasMultiSourceJammingFacts(const session::EnvironmentSnapshot& environment_snapshot) {
  return !environment_snapshot.jammer_sources.empty();
}

bool TryResolveEngineeringInterferencePowerW(
    const ExecutionConfig& config, const session::EnvironmentSnapshot& environment_snapshot,
    float* received_power_w) {
  if (received_power_w == nullptr) {
    return false;
  }
  if (environment_snapshot.interference_mode !=
      oneq::electromagnetics::RfInterferenceMode::kEngineering) {
    *received_power_w = 0.0f;
    return true;
  }
  oneq::electromagnetics::RfReceiverSite receiver;
  if (!TryBuildReceiverSite(config, environment_snapshot, &receiver)) {
    return false;
  }
  oneq::electromagnetics::RfLinkEvaluationConfig link_config;
  link_config.additional_propagation_loss_db =
      std::max(0.0, 0.5 * static_cast<double>(environment_snapshot.atmospheric_physics_loss_db));
  std::vector<oneq::electromagnetics::RfLinkResult> links;
  links.reserve(environment_snapshot.engineering_interference_emissions.size());
  for (const oneq::electromagnetics::RfEmission& emission :
       environment_snapshot.engineering_interference_emissions) {
    oneq::electromagnetics::RfLinkResult link;
    if (!oneq::electromagnetics::TryEvaluateRfLink(emission, receiver, link_config, &link)) {
      return false;
    }
    links.push_back(link);
  }
  double total_received_power_w = 0.0;
  if (!oneq::electromagnetics::TryAggregateRfReceivedPower(links, &total_received_power_w) ||
      total_received_power_w > static_cast<double>(std::numeric_limits<float>::max()) ||
      total_received_power_w >
          static_cast<double>(config.detection.engineering.receiver.maximum_linear_input_power_w)) {
    return false;
  }
  *received_power_w = static_cast<float>(total_received_power_w);
  return true;
}

float ComputeResidualJammerFactor(const session::ArControlProfile& control_profile,
                                  const session::JammerSourceFact& jammer_source) {
  float residual_factor = 1.0f;

  if (control_profile.enable_sidelobe_canceller && jammer_source.in_sidelobe) {
    residual_factor *= kSidelobeCancellerResidual;
  }
  if (control_profile.enable_adaptive_beamforming) {
    residual_factor = residual_factor * (jammer_source.angular_span_deg > 0.0f &&
                                                 jammer_source.angular_span_deg <= 10.0f
                                             ? kAdaptiveBeamNarrowResidual
                                             : kAdaptiveBeamWideResidual);
  }
  if (control_profile.enable_agility_frequency && jammer_source.frequency_overlap_ratio > 0.0f) {
    residual_factor *= utils::ClampFloat(1.0f - 0.50f * jammer_source.frequency_overlap_ratio,
                                         kAgilityFreqResidualMin, 1.0f);
  }
  if (control_profile.enable_eccm_rejitter && jammer_source.prf_lock_risk > 0.0f) {
    residual_factor *= utils::ClampFloat(1.0f - 0.55f * jammer_source.prf_lock_risk,
                                         kEccmRejitterResidualMin, 1.0f);
  }
  if (control_profile.eccm_burnthrough_gain > 1.0f) {
    residual_factor *= utils::ClampFloat(1.0f / control_profile.eccm_burnthrough_gain,
                                         kBurnthroughResidualMin, 1.0f);
  }

  switch (jammer_source.technique) {
    case config::JammingTechnique::kNoiseSuppression:
      if (control_profile.enable_sidelobe_canceller && jammer_source.in_sidelobe) {
        residual_factor *= kNoiseSidelobeCancellerFactor;
      }
      break;
    case config::JammingTechnique::kDeception:
      if (control_profile.enable_agility_frequency) {
        residual_factor *= kDeceptionAgilityFactor;
      }
      if (control_profile.enable_eccm_rejitter) {
        residual_factor *= kDeceptionRejitterFactor;
      }
      break;
    case config::JammingTechnique::kRepeater:
      if (control_profile.enable_eccm_rejitter) {
        residual_factor *= kRepeaterRejitterFactor;
      }
      break;
    case config::JammingTechnique::kUnknown:
    default:
      break;
  }

  return utils::ClampFloat(residual_factor, kResidualFactorMin, 1.0f);
}

float ComputeLegacySourceJamToNoiseRatio(const JammingEffectsConfig& cfg,
                                         const session::JammerSourceFact& jammer_source) {
  float source_weight = 1.0f;
  switch (jammer_source.technique) {
    case config::JammingTechnique::kNoiseSuppression:
      source_weight = kPhysicalWeightNoise;
      break;
    case config::JammingTechnique::kDeception:
      source_weight = kPhysicalWeightDeception;
      break;
    case config::JammingTechnique::kRepeater:
      source_weight = kPhysicalWeightRepeater;
      break;
    case config::JammingTechnique::kUnknown:
    default:
      source_weight = kPhysicalWeightUnknown;
      break;
  }
  return utils::DbToLinearPower(jammer_source.power_db) * source_weight *
         ResolveJammerConfidenceWeight(cfg, jammer_source);
}

config::JammingSemantic ResolveDominantJammingSemantic(
    const session::ArControlProfile& control_profile,
    const session::EnvironmentSnapshot& environment_snapshot) {
  if (!environment_snapshot.jamming_detected) {
    return config::JammingSemantic::kNone;
  }

  if (!HasMultiSourceJammingFacts(environment_snapshot)) {
    return config::JammingSemantic::kNone;
  }

  float type_scores[3] = {0.0f, 0.0f, 0.0f};
  for (std::size_t i = 0; i < environment_snapshot.jammer_sources.size(); ++i) {
    const session::JammerSourceFact& source = environment_snapshot.jammer_sources[i];
    const float contribution = ComputeTrackLevelJammingContribution(control_profile, source);
    switch (source.technique) {
      case config::JammingTechnique::kNoiseSuppression:
        type_scores[0] += contribution;
        break;
      case config::JammingTechnique::kDeception:
        type_scores[1] += contribution;
        break;
      case config::JammingTechnique::kRepeater:
        type_scores[2] += contribution;
        break;
      case config::JammingTechnique::kUnknown:
      default:
        type_scores[0] += kUnknownNoiseSplitWeight * contribution;
        type_scores[1] += kUnknownNoiseSplitWeight * contribution;
        break;
    }
  }

  std::size_t best_index = 0U;
  std::size_t second_index = 1U;
  for (std::size_t i = 1; i < 3U; ++i) {
    if (type_scores[i] > type_scores[best_index]) {
      second_index = best_index;
      best_index = i;
    } else if (i != best_index && type_scores[i] > type_scores[second_index]) {
      second_index = i;
    }
  }

  if (type_scores[best_index] <= 1e-6f) {
    return config::JammingSemantic::kNone;
  }
  if (second_index != best_index &&
      type_scores[second_index] >= kMixedSemanticDominanceRatio * type_scores[best_index] &&
      type_scores[second_index] > kMixedSemanticMinScore) {
    return config::JammingSemantic::kMixed;
  }

  if (best_index == 0U) {
    return config::JammingSemantic::kNoiseSuppression;
  }
  if (best_index == 1U) {
    return config::JammingSemantic::kDeception;
  }
  return config::JammingSemantic::kRepeater;
}

float ComputeTrackLevelJammingSeverity(const session::ArControlProfile& control_profile,
                                       const session::EnvironmentSnapshot& environment_snapshot) {
  if (!environment_snapshot.jamming_detected) {
    return 0.0f;
  }

  if (!HasMultiSourceJammingFacts(environment_snapshot)) {
    return 0.0f;
  }

  float total_severity = 0.0f;
  for (std::size_t i = 0; i < environment_snapshot.jammer_sources.size(); ++i) {
    total_severity += ComputeTrackLevelJammingContribution(control_profile,
                                                           environment_snapshot.jammer_sources[i]);
  }
  return utils::ClampFloat(total_severity, 0.0f, 1.0f);
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
