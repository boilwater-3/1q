#include "airborne_radar/signal/detection/ArInterferenceObservationResolver.h"

#include <algorithm>
#include <cmath>
#include <tuple>

namespace airborne_radar {
namespace signal {
namespace detection {
namespace {

constexpr double kRadiansToDegrees = 57.2957795130823208768;

bool SameIdentity(const oneq::electromagnetics::RfEmissionIdentity& left,
                  const oneq::electromagnetics::RfEmissionIdentity& right) {
  return left.platform_id == right.platform_id && left.equipment_id == right.equipment_id &&
         left.emission_id == right.emission_id;
}

const oneq::electromagnetics::RfSceneEmission* FindEmission(
    const oneq::electromagnetics::RfSceneFrame& scene,
    const oneq::electromagnetics::RfEmissionIdentity& identity) {
  for (const auto& emission : scene.emissions) {
    if (SameIdentity(emission.identity, identity)) {
      return &emission;
    }
  }
  return nullptr;
}

double CenterFrequencyHz(const oneq::electromagnetics::RfWaveformSchedule& waveform) {
  if (waveform.kind == oneq::electromagnetics::RfSceneWaveformKind::kLinearSweep) {
    return 0.5 * (waveform.sweep_start_frequency_hz + waveform.sweep_stop_frequency_hz);
  }
  return waveform.center_frequency_hz;
}

double ObservableBandwidthHz(const oneq::electromagnetics::RfWaveformSchedule& waveform) {
  if (waveform.kind == oneq::electromagnetics::RfSceneWaveformKind::kLinearSweep) {
    return std::fabs(waveform.sweep_stop_frequency_hz - waveform.sweep_start_frequency_hz) +
           waveform.occupied_bandwidth_hz;
  }
  return waveform.occupied_bandwidth_hz;
}

using ObservationSortKey =
    std::tuple<double, double, double, double, double, std::uint8_t, double>;

ObservationSortKey MakeSortKey(const session::ArInterferenceObservation& observation) {
  return std::make_tuple(
      observation.estimated_bearing_azimuth_deg, observation.estimated_bearing_elevation_deg,
      observation.estimated_off_boresight_deg, observation.estimated_center_frequency_hz,
      observation.estimated_bandwidth_hz,
      static_cast<std::uint8_t>(observation.estimated_waveform_kind),
      observation.jammer_to_noise_db);
}

}  // namespace

bool TryResolveArInterferenceObservations(
    const oneq::electromagnetics::RfSceneFrame& scene,
    const oneq::electromagnetics::RfSceneReceiverState& receiver,
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& incident_links,
    double thermal_noise_power_w, double jammer_to_noise_gate_db,
    std::vector<session::ArInterferenceObservation>* observations) {
  if (observations == nullptr || !oneq::electromagnetics::TryValidateRfSceneFrame(scene) ||
      !std::isfinite(thermal_noise_power_w) || thermal_noise_power_w <= 0.0 ||
      !std::isfinite(jammer_to_noise_gate_db)) {
    return false;
  }

  std::vector<session::ArInterferenceObservation> candidate;
  candidate.reserve(incident_links.size());
  for (const auto& link : incident_links) {
    if (SameIdentity(link.identity, own_emission_identity)) {
      continue;
    }
    if (!std::isfinite(link.received_power_w) || link.received_power_w < 0.0) {
      return false;
    }
    const double jammer_to_noise_linear = link.received_power_w / thermal_noise_power_w;
    if (jammer_to_noise_linear <= 0.0) {
      continue;
    }
    const double jammer_to_noise_db = 10.0 * std::log10(jammer_to_noise_linear);
    if (jammer_to_noise_db < jammer_to_noise_gate_db) {
      continue;
    }
    const oneq::electromagnetics::RfSceneEmission* emission = FindEmission(scene, link.identity);
    if (emission == nullptr) {
      return false;
    }
    const double x = emission->position_ecef_m.x_m - receiver.position_ecef_m.x_m;
    const double y = emission->position_ecef_m.y_m - receiver.position_ecef_m.y_m;
    const double z = emission->position_ecef_m.z_m - receiver.position_ecef_m.z_m;
    const double range_m = std::sqrt(x * x + y * y + z * z);
    if (!std::isfinite(range_m) || range_m <= 0.0) {
      return false;
    }
    session::ArInterferenceObservation observation;
    observation.estimated_bearing_azimuth_deg = std::atan2(y, x) * kRadiansToDegrees;
    observation.estimated_bearing_elevation_deg = std::asin(z / range_m) * kRadiansToDegrees;
    const double direction_x = x / range_m;
    const double direction_y = y / range_m;
    const double direction_z = z / range_m;
    const double boresight_dot =
        std::max(-1.0, std::min(1.0, direction_x * receiver.antenna.boresight_ecef.x +
                                        direction_y * receiver.antenna.boresight_ecef.y +
                                        direction_z * receiver.antenna.boresight_ecef.z));
    observation.estimated_off_boresight_deg = std::acos(boresight_dot) * kRadiansToDegrees;
    observation.estimated_center_frequency_hz = CenterFrequencyHz(emission->waveform);
    observation.estimated_bandwidth_hz = ObservableBandwidthHz(emission->waveform);
    observation.estimated_waveform_kind = emission->waveform.kind;
    observation.jammer_to_noise_db = jammer_to_noise_db;
    const double quality_scale = std::sqrt(std::max(1.0, jammer_to_noise_linear));
    observation.bearing_standard_deviation_deg =
        receiver.antenna.half_power_beamwidth_deg / quality_scale;
    observation.frequency_standard_deviation_hz =
        emission->waveform.occupied_bandwidth_hz / (2.0 * quality_scale);
    observation.bandwidth_standard_deviation_hz =
        observation.estimated_bandwidth_hz / quality_scale;
    candidate.push_back(observation);
  }
  std::sort(candidate.begin(), candidate.end(), [](const auto& left, const auto& right) {
    return MakeSortKey(left) < MakeSortKey(right);
  });
  for (std::size_t index = 0U; index < candidate.size(); ++index) {
    candidate[index].observation_id = static_cast<std::uint64_t>(index + 1U);
  }
  *observations = candidate;
  return true;
}

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar
