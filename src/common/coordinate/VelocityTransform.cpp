#include "1q/coordinate/velocity_transform.h"

#include <cmath>

#include "1q/coordinate/position_transform.h"

namespace oneq {
namespace coordinate {

namespace {

constexpr double kPi = 3.14159265358979323846;

bool IsFiniteScalar(double value) { return std::isfinite(value) != 0; }
double DegToRad(double deg) { return deg * kPi / 180.0; }

}  // namespace

bool IsFinite(const EcefVelocityMps& velocity) {
  return IsFiniteScalar(velocity.x_mps) && IsFiniteScalar(velocity.y_mps) &&
         IsFiniteScalar(velocity.z_mps);
}

bool IsFinite(const EnuVelocityMps& velocity) {
  return IsFiniteScalar(velocity.east_mps) && IsFiniteScalar(velocity.north_mps) &&
         IsFiniteScalar(velocity.up_mps);
}

bool IsFinite(const NedVelocityMps& velocity) {
  return IsFiniteScalar(velocity.north_mps) && IsFiniteScalar(velocity.east_mps) &&
         IsFiniteScalar(velocity.down_mps);
}

bool IsFinite(const NueVelocityMps& velocity) {
  return IsFiniteScalar(velocity.north_mps) && IsFiniteScalar(velocity.up_mps) &&
         IsFiniteScalar(velocity.east_mps);
}

bool TryEcefToEnuVelocity(const EcefVelocityMps& ecef_velocity,
                          const LlaPositionDegM& origin_lla,
                          EnuVelocityMps* enu_velocity) {
  if (enu_velocity == nullptr || !IsFinite(ecef_velocity) || !IsValid(origin_lla)) {
    return false;
  }

  const double lat_rad = DegToRad(origin_lla.latitude_deg);
  const double lon_rad = DegToRad(origin_lla.longitude_deg);
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);

  enu_velocity->east_mps = -sin_lon * ecef_velocity.x_mps + cos_lon * ecef_velocity.y_mps;
  enu_velocity->north_mps = -sin_lat * cos_lon * ecef_velocity.x_mps -
                            sin_lat * sin_lon * ecef_velocity.y_mps +
                            cos_lat * ecef_velocity.z_mps;
  enu_velocity->up_mps = cos_lat * cos_lon * ecef_velocity.x_mps +
                         cos_lat * sin_lon * ecef_velocity.y_mps +
                         sin_lat * ecef_velocity.z_mps;
  return IsFinite(*enu_velocity);
}

bool TryEcefToNedVelocity(const EcefVelocityMps& ecef_velocity,
                          const LlaPositionDegM& origin_lla,
                          NedVelocityMps* ned_velocity) {
  if (ned_velocity == nullptr) {
    return false;
  }
  EnuVelocityMps enu_velocity;
  if (!TryEcefToEnuVelocity(ecef_velocity, origin_lla, &enu_velocity)) {
    return false;
  }
  *ned_velocity = ToNedVelocity(enu_velocity);
  return true;
}

bool TryEcefToNueVelocity(const EcefVelocityMps& ecef_velocity,
                          const LlaPositionDegM& origin_lla,
                          NueVelocityMps* nue_velocity) {
  if (nue_velocity == nullptr) {
    return false;
  }
  EnuVelocityMps enu_velocity;
  if (!TryEcefToEnuVelocity(ecef_velocity, origin_lla, &enu_velocity)) {
    return false;
  }
  *nue_velocity = ToNueVelocity(enu_velocity);
  return true;
}

NedVelocityMps ToNedVelocity(const EnuVelocityMps& enu_velocity) {
  NedVelocityMps ned_velocity;
  ned_velocity.north_mps = enu_velocity.north_mps;
  ned_velocity.east_mps = enu_velocity.east_mps;
  ned_velocity.down_mps = -enu_velocity.up_mps;
  return ned_velocity;
}

EnuVelocityMps ToEnuVelocity(const NedVelocityMps& ned_velocity) {
  EnuVelocityMps enu_velocity;
  enu_velocity.east_mps = ned_velocity.east_mps;
  enu_velocity.north_mps = ned_velocity.north_mps;
  enu_velocity.up_mps = -ned_velocity.down_mps;
  return enu_velocity;
}

NueVelocityMps ToNueVelocity(const EnuVelocityMps& enu_velocity) {
  NueVelocityMps nue_velocity;
  nue_velocity.north_mps = enu_velocity.north_mps;
  nue_velocity.up_mps = enu_velocity.up_mps;
  nue_velocity.east_mps = enu_velocity.east_mps;
  return nue_velocity;
}

EnuVelocityMps ToEnuVelocity(const NueVelocityMps& nue_velocity) {
  EnuVelocityMps enu_velocity;
  enu_velocity.east_mps = nue_velocity.east_mps;
  enu_velocity.north_mps = nue_velocity.north_mps;
  enu_velocity.up_mps = nue_velocity.up_mps;
  return enu_velocity;
}

NueVelocityMps ToNueVelocity(const NedVelocityMps& ned_velocity) {
  NueVelocityMps nue_velocity;
  nue_velocity.north_mps = ned_velocity.north_mps;
  nue_velocity.up_mps = -ned_velocity.down_mps;
  nue_velocity.east_mps = ned_velocity.east_mps;
  return nue_velocity;
}

NedVelocityMps ToNedVelocity(const NueVelocityMps& nue_velocity) {
  NedVelocityMps ned_velocity;
  ned_velocity.north_mps = nue_velocity.north_mps;
  ned_velocity.east_mps = nue_velocity.east_mps;
  ned_velocity.down_mps = -nue_velocity.up_mps;
  return ned_velocity;
}

}  // namespace coordinate
}  // namespace oneq
