#include "1q/airborne_radar/session/RadarExternalOutputAdapter.h"

#include <cmath>

#include "1q/coordinate/attitude_transform.h"
#include "1q/coordinate/position_transform.h"
#include "1q/coordinate/velocity_transform.h"

namespace airborne_radar {
namespace session {

namespace {

constexpr double kPi = 3.14159265358979323846;

bool IsFinite(float value) { return std::isfinite(value) != 0; }

bool IsFiniteVector3f(const oneq::foundation::Vector3f& value) {
  return IsFinite(value.x) && IsFinite(value.y) && IsFinite(value.z);
}

double DegToRad(double deg) { return deg * kPi / 180.0; }

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

bool TryEnuPositionToEcef(const oneq::coordinate::Vector3d& enu,
                          const oneq::coordinate::LlaPositionDegM& origin_lla,
                          oneq::coordinate::EcefPositionM* ecef) {
  if (ecef == nullptr || !oneq::coordinate::IsValid(origin_lla)) {
    return false;
  }

  oneq::coordinate::EcefPositionM origin_ecef;
  if (!oneq::coordinate::TryLlaToEcef(origin_lla, &origin_ecef)) {
    return false;
  }

  const double lat_rad = DegToRad(origin_lla.latitude_deg);
  const double lon_rad = DegToRad(origin_lla.longitude_deg);
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);

  ecef->x_m =
      origin_ecef.x_m - sin_lon * enu.x - sin_lat * cos_lon * enu.y + cos_lat * cos_lon * enu.z;
  ecef->y_m =
      origin_ecef.y_m + cos_lon * enu.x - sin_lat * sin_lon * enu.y + cos_lat * sin_lon * enu.z;
  ecef->z_m = origin_ecef.z_m + cos_lat * enu.y + sin_lat * enu.z;
  return oneq::coordinate::IsFinite(*ecef);
}

bool TryEnuVelocityToEcef(const oneq::coordinate::Vector3d& enu_velocity,
                          const oneq::coordinate::LlaPositionDegM& origin_lla,
                          oneq::coordinate::EcefVelocityMps* ecef_velocity) {
  if (ecef_velocity == nullptr || !oneq::coordinate::IsValid(origin_lla)) {
    return false;
  }

  const double lat_rad = DegToRad(origin_lla.latitude_deg);
  const double lon_rad = DegToRad(origin_lla.longitude_deg);
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);

  ecef_velocity->x_mps = -sin_lon * enu_velocity.x - sin_lat * cos_lon * enu_velocity.y +
                         cos_lat * cos_lon * enu_velocity.z;
  ecef_velocity->y_mps = cos_lon * enu_velocity.x - sin_lat * sin_lon * enu_velocity.y +
                         cos_lat * sin_lon * enu_velocity.z;
  ecef_velocity->z_mps = cos_lat * enu_velocity.y + sin_lat * enu_velocity.z;
  return oneq::coordinate::IsFinite(*ecef_velocity);
}

}  // namespace

bool TryMakeExternalTrackFromSnapshot(const model::TrackStateSnapshot& snapshot,
                                      const RadarLocalFrameReference& reference,
                                      oneq::foundation::Vector3f radar_local_velocity_mps,
                                      RadarExternalTrackKinematics* output) {
  if (output == nullptr || !IsFiniteVector3f(radar_local_velocity_mps) ||
      !IsFinite(snapshot.position_x) || !IsFinite(snapshot.position_y) ||
      !IsFinite(snapshot.position_z) || !IsFinite(snapshot.velocity_x) ||
      !IsFinite(snapshot.velocity_y) || !IsFinite(snapshot.velocity_z)) {
    return false;
  }

  const oneq::coordinate::Vector3d position_enu = RotateLocalToEnu(
      static_cast<double>(snapshot.position_x), static_cast<double>(snapshot.position_y),
      static_cast<double>(snapshot.position_z), reference.radar_attitude_deg);
  oneq::coordinate::EcefPositionM position_ecef;
  if (!TryEnuPositionToEcef(position_enu, reference.origin_lla, &position_ecef)) {
    return false;
  }

  const double absolute_velocity_local_x =
      static_cast<double>(snapshot.velocity_x) + static_cast<double>(radar_local_velocity_mps.x);
  const double absolute_velocity_local_y =
      static_cast<double>(snapshot.velocity_y) + static_cast<double>(radar_local_velocity_mps.y);
  const double absolute_velocity_local_z =
      static_cast<double>(snapshot.velocity_z) + static_cast<double>(radar_local_velocity_mps.z);
  const oneq::coordinate::Vector3d velocity_enu =
      RotateLocalToEnu(absolute_velocity_local_x, absolute_velocity_local_y,
                       absolute_velocity_local_z, reference.radar_attitude_deg);
  oneq::coordinate::EcefVelocityMps velocity_ecef;
  if (!TryEnuVelocityToEcef(velocity_enu, reference.origin_lla, &velocity_ecef)) {
    return false;
  }

  output->association_key = snapshot.association_key;
  output->external_target_id = snapshot.external_target_id;
  output->status = snapshot.status;
  output->target_position_ecef_m = position_ecef;
  output->target_velocity_mps = velocity_ecef;
  output->speed = snapshot.speed;
  output->rcs = snapshot.rcs;
  output->jamming_detected = snapshot.jamming_detected;
  output->hit_count = snapshot.hit_count;
  output->miss_count = snapshot.miss_count;
  output->target_probability = snapshot.target_probability;
  return true;
}

}  // namespace session
}  // namespace airborne_radar
