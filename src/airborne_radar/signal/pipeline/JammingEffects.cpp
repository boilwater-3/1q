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

namespace {

constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;

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

}  // namespace

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

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
