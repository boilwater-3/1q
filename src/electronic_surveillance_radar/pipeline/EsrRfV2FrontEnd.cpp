#include "electronic_surveillance_radar/pipeline/EsrRfV2FrontEnd.h"

#include <algorithm>
#include <cmath>

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace {

bool IsFinite(double value) { return std::isfinite(value) != 0; }

bool IdentityLess(const oneq::electromagnetics::RfIncidentLinkResult& left,
                  const oneq::electromagnetics::RfIncidentLinkResult& right) {
  if (left.identity.platform_id != right.identity.platform_id) {
    return left.identity.platform_id < right.identity.platform_id;
  }
  if (left.identity.equipment_id != right.identity.equipment_id) {
    return left.identity.equipment_id < right.identity.equipment_id;
  }
  return left.identity.emission_id < right.identity.emission_id;
}

bool TryResolveBoresight(const session::EsrCycleInput& input, double beam_az_deg,
                         double beam_el_deg,
                         oneq::electromagnetics::RfSceneDirection* boresight) {
  if (boresight == nullptr || !IsFinite(beam_az_deg) || !IsFinite(beam_el_deg) ||
      beam_az_deg < -180.0 || beam_az_deg > 180.0 || beam_el_deg < -90.0 ||
      beam_el_deg > 90.0) {
    return false;
  }
  constexpr double kPi = 3.14159265358979323846;
  const double azimuth_rad = beam_az_deg * kPi / 180.0;
  const double elevation_rad = beam_el_deg * kPi / 180.0;
  const double cos_elevation = std::cos(elevation_rad);
  oneq::coordinate::EulerAnglesDeg attitude;
  attitude.yaw_deg = input.platform_pose.attitude_deg.yaw_deg;
  attitude.pitch_deg = input.platform_pose.attitude_deg.pitch_deg;
  attitude.roll_deg = input.platform_pose.attitude_deg.roll_deg;
  const oneq::coordinate::Vector3d enu_direction = oneq::coordinate::RotateLocalToEnu(
      cos_elevation * std::cos(azimuth_rad), cos_elevation * std::sin(azimuth_rad),
      std::sin(elevation_rad), attitude);
  oneq::coordinate::LlaPositionDegM platform_lla;
  oneq::coordinate::Vector3d ecef_direction;
  if (!oneq::coordinate::TryEcefToLla(input.platform_position_ecef_m, &platform_lla) ||
      !oneq::coordinate::TryEnuToEcefDirection(enu_direction, platform_lla, &ecef_direction)) {
    return false;
  }
  boresight->x = ecef_direction.x;
  boresight->y = ecef_direction.y;
  boresight->z = ecef_direction.z;
  return true;
}

}  // namespace

bool TryResolveEsrRfV2FrontEnd(const session::EsrCycleInput& input,
                               const config::EsrHardwareConfig& hardware,
                               double beam_az_deg, double beam_el_deg,
                               double receiver_center_frequency_hz,
                               double receiver_bandwidth_hz,
                               double additional_propagation_loss_db,
                               EsrRfV2FrontEndResult* result) {
  if (result == nullptr ||
      !input.has_platform_ecef_kinematics || input.platform_entity_id == 0U ||
      hardware.receiver_equipment_id == 0U || !IsFinite(receiver_center_frequency_hz) ||
      receiver_center_frequency_hz <= 0.0 || !IsFinite(receiver_bandwidth_hz) ||
      receiver_bandwidth_hz <= 0.0 || !IsFinite(additional_propagation_loss_db) ||
      additional_propagation_loss_db < 0.0 ||
      !oneq::electromagnetics::TryValidateRfSceneFrame(input.interference) ||
      input.interference.world_cycle_index != input.cycle_index ||
      input.interference.window_start_time_s != input.cycle_start_time_s ||
      input.interference.window_duration_s != static_cast<double>(input.dt_sec)) {
    return false;
  }

  EsrRfV2FrontEndResult candidate;
  oneq::electromagnetics::RfSceneReceiverState& receiver = candidate.receiver;
  receiver.platform_id = input.platform_entity_id;
  receiver.equipment_id = hardware.receiver_equipment_id;
  receiver.position_ecef_m = input.platform_position_ecef_m;
  receiver.velocity_ecef_mps = input.platform_velocity_ecef_mps;
  receiver.window_start_time_s = input.cycle_start_time_s;
  receiver.window_duration_s = static_cast<double>(input.dt_sec);
  receiver.center_frequency_hz = receiver_center_frequency_hz;
  receiver.bandwidth_hz = receiver_bandwidth_hz;
  receiver.receiver_system_loss_db = static_cast<double>(hardware.integrated_receive_loss_db);
  receiver.minimum_far_field_range_m = static_cast<double>(hardware.minimum_far_field_range_m);
  receiver.antenna.peak_gain_dbi = static_cast<double>(hardware.antenna_peak_gain_dbi);
  receiver.antenna.half_power_beamwidth_deg =
      std::max(static_cast<double>(hardware.beam_az_width_deg),
               static_cast<double>(hardware.beam_el_width_deg));
  receiver.antenna.sidelobe_level_db = static_cast<double>(hardware.antenna_sidelobe_level_db);
  receiver.antenna.backlobe_level_db = static_cast<double>(hardware.antenna_backlobe_level_db);
  receiver.antenna.cross_polarization_isolation_db =
      static_cast<double>(hardware.cross_polarization_isolation_db);
  receiver.polarization = hardware.polarization;
  receiver.co_site_paths.reserve(hardware.co_site_paths.size());
  for (const config::EsrCoSiteIsolationPath& path : hardware.co_site_paths) {
    oneq::electromagnetics::RfCoSiteIsolationPath resolved_path;
    resolved_path.transmitter_equipment_id = path.transmitter_equipment_id;
    resolved_path.receiver_equipment_id = hardware.receiver_equipment_id;
    resolved_path.isolation_db = path.isolation_db;
    receiver.co_site_paths.push_back(resolved_path);
  }
  if (!TryResolveBoresight(input,
                           beam_az_deg + static_cast<double>(hardware.antenna_mount_az_deg),
                           beam_el_deg + static_cast<double>(hardware.antenna_mount_el_deg),
                           &receiver.antenna.boresight_ecef)) {
    return false;
  }

  oneq::electromagnetics::RfIncidentLinkConfig link_config;
  link_config.additional_propagation_loss_db = additional_propagation_loss_db;
  candidate.incident_links.reserve(input.interference.emissions.size());
  for (const oneq::electromagnetics::RfSceneEmission& emission :
       input.interference.emissions) {
    oneq::electromagnetics::RfIncidentLinkResult link;
    if (!oneq::electromagnetics::TryEvaluateRfIncidentLink(emission, receiver, link_config,
                                                           &link)) {
      return false;
    }
    candidate.incident_links.push_back(link);
  }
  std::sort(candidate.incident_links.begin(), candidate.incident_links.end(), IdentityLess);
  if (!oneq::electromagnetics::TryAggregateRfIncidentPower(candidate.incident_links,
                                                           &candidate.total_incident_power_w)) {
    return false;
  }
  candidate.receiver_saturated =
      candidate.total_incident_power_w >
      static_cast<double>(hardware.maximum_linear_input_power_w);
  *result = candidate;
  return true;
}

}  // namespace pipeline
}  // namespace electronic_surveillance_radar
