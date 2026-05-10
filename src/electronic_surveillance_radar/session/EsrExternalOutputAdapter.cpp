#include "1q/electronic_surveillance_radar/session/EsrExternalOutputAdapter.h"

#include <cmath>

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"

namespace electronic_surveillance_radar {
namespace session {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kNormFloor = 1.0e-12;

bool IsFinite(double value) { return std::isfinite(value) != 0; }
double DegToRad(double deg) { return deg * kPi / 180.0; }

oneq::coordinate::EulerAnglesDeg ToCoordinateEuler(const EsrEulerAngleDeg& attitude_deg) {
  oneq::coordinate::EulerAnglesDeg output;
  output.yaw_deg = static_cast<double>(attitude_deg.yaw_deg);
  output.pitch_deg = static_cast<double>(attitude_deg.pitch_deg);
  output.roll_deg = static_cast<double>(attitude_deg.roll_deg);
  return output;
}

oneq::coordinate::Vector3d RotateLocalToEnu(
    double local_x, double local_y, double local_z,
    const oneq::coordinate::EulerAnglesDeg& local_attitude_deg) {
  const oneq::coordinate::RotationMatrix3d rotation =
      oneq::coordinate::BuildRotationMatrix(local_attitude_deg);
  oneq::coordinate::Vector3d enu;
  enu.x = rotation.m00 * local_x + rotation.m01 * local_y + rotation.m02 * local_z;
  enu.y = rotation.m10 * local_x + rotation.m11 * local_y + rotation.m12 * local_z;
  enu.z = rotation.m20 * local_x + rotation.m21 * local_y + rotation.m22 * local_z;
  return enu;
}

oneq::coordinate::Vector3d BearingVectorFromAngles(double az_deg, double el_deg) {
  const double az_rad = DegToRad(az_deg);
  const double el_rad = DegToRad(el_deg);
  const double horizontal = std::cos(el_rad);
  oneq::coordinate::Vector3d vector;
  vector.x = horizontal * std::cos(az_rad);
  vector.y = horizontal * std::sin(az_rad);
  vector.z = std::sin(el_rad);
  return vector;
}

bool TryEnuUnitToEcefUnit(const oneq::coordinate::Vector3d& enu_unit,
                          const oneq::coordinate::LlaPositionDegM& origin_lla,
                          oneq::coordinate::Vector3d* ecef_unit) {
  if (ecef_unit == nullptr || !oneq::coordinate::IsValid(origin_lla)) {
    return false;
  }

  const double lat_rad = DegToRad(origin_lla.latitude_deg);
  const double lon_rad = DegToRad(origin_lla.longitude_deg);
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);

  ecef_unit->x =
      -sin_lon * enu_unit.x - sin_lat * cos_lon * enu_unit.y + cos_lat * cos_lon * enu_unit.z;
  ecef_unit->y =
      cos_lon * enu_unit.x - sin_lat * sin_lon * enu_unit.y + cos_lat * sin_lon * enu_unit.z;
  ecef_unit->z = cos_lat * enu_unit.y + sin_lat * enu_unit.z;
  const double norm = std::sqrt(ecef_unit->x * ecef_unit->x + ecef_unit->y * ecef_unit->y +
                                ecef_unit->z * ecef_unit->z);
  if (norm <= kNormFloor) {
    return false;
  }
  ecef_unit->x /= norm;
  ecef_unit->y /= norm;
  ecef_unit->z /= norm;
  return true;
}

bool TryBearingToEcefUnit(double az_deg, double el_deg, const EsrCoordinateReference& reference,
                          const EsrPoseState& platform_pose,
                          oneq::coordinate::Vector3d* ecef_unit) {
  if (!IsFinite(az_deg) || !IsFinite(el_deg)) {
    return false;
  }
  const oneq::coordinate::Vector3d platform_frame = BearingVectorFromAngles(az_deg, el_deg);
  const oneq::coordinate::Vector3d local =
      RotateLocalToEnu(platform_frame.x, platform_frame.y, platform_frame.z,
                       ToCoordinateEuler(platform_pose.attitude_deg));
  const oneq::coordinate::Vector3d enu =
      RotateLocalToEnu(local.x, local.y, local.z, reference.frame_attitude_deg);
  return TryEnuUnitToEcefUnit(enu, reference.origin_lla, ecef_unit);
}

}  // namespace

bool TryMakeExternalObservationFromRecord(const model::EmitterObservation& observation,
                                          const EsrCoordinateReference& reference,
                                          const EsrPoseState& platform_pose,
                                          EsrExternalObservation* output) {
  if (output == nullptr) {
    return false;
  }
  oneq::coordinate::Vector3d bearing_unit_ecef;
  if (!TryBearingToEcefUnit(observation.aoa_az_deg, observation.aoa_el_deg, reference,
                            platform_pose, &bearing_unit_ecef)) {
    return false;
  }

  output->observation_id = observation.observation_id;
  output->timestamp_s = observation.timestamp_s;
  output->bearing_unit_ecef = bearing_unit_ecef;
  output->aoa_az_deg = observation.aoa_az_deg;
  output->aoa_el_deg = observation.aoa_el_deg;
  output->rf_hz = observation.rf_hz;
  output->pulse_width_s = observation.pulse_width_s;
  output->amplitude_db = observation.amplitude_db;
  output->snr_db = observation.snr_db;
  output->quality = observation.quality;
  output->is_jammed = observation.is_jammed;
  return true;
}

bool TryMakeExternalHypothesisFromRecord(const model::EmitterHypothesis& hypothesis,
                                         const EsrCoordinateReference& reference,
                                         const EsrPoseState& platform_pose,
                                         EsrExternalEmitterHypothesis* output) {
  if (output == nullptr) {
    return false;
  }
  oneq::coordinate::Vector3d bearing_unit_ecef;
  if (!TryBearingToEcefUnit(static_cast<double>(hypothesis.bearing_az_deg),
                            static_cast<double>(hypothesis.bearing_el_deg), reference,
                            platform_pose, &bearing_unit_ecef)) {
    return false;
  }

  output->hypothesis_id = hypothesis.hypothesis_id;
  output->bearing_unit_ecef = bearing_unit_ecef;
  output->bearing_az_deg = hypothesis.bearing_az_deg;
  output->bearing_el_deg = hypothesis.bearing_el_deg;
  output->bearing_std_deg = hypothesis.bearing_std_deg;
  output->confidence = hypothesis.confidence;
  output->last_seen_cycle = hypothesis.last_seen_cycle;
  output->mode = hypothesis.mode;
  output->threat_level = hypothesis.threat_level;
  return true;
}

}  // namespace session
}  // namespace electronic_surveillance_radar
