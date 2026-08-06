#include "1q/coordinate/velocity_transform.h"

#include <cmath>
#include "common/validation/ValidationUtils.h"
#include "common/numerics/Constants.h"

#include "1q/coordinate/position_transform.h"

namespace oneq {
namespace coordinate {

namespace {



}  // namespace

bool IsFinite(const EcefVelocityMps& velocity) {
  return oneq::common::validation::IsFinite(velocity.x_mps) && oneq::common::validation::IsFinite(velocity.y_mps) &&
         oneq::common::validation::IsFinite(velocity.z_mps);
}

bool IsFinite(const EnuVelocityMps& velocity) {
  return oneq::common::validation::IsFinite(velocity.east_mps) && oneq::common::validation::IsFinite(velocity.north_mps) &&
         oneq::common::validation::IsFinite(velocity.up_mps);
}

bool IsFinite(const NedVelocityMps& velocity) {
  return oneq::common::validation::IsFinite(velocity.north_mps) && oneq::common::validation::IsFinite(velocity.east_mps) &&
         oneq::common::validation::IsFinite(velocity.down_mps);
}

bool IsFinite(const NueVelocityMps& velocity) {
  return oneq::common::validation::IsFinite(velocity.north_mps) && oneq::common::validation::IsFinite(velocity.up_mps) &&
         oneq::common::validation::IsFinite(velocity.east_mps);
}

bool TryEcefToEnuVelocity(const EcefVelocityMps& ecef_velocity,
                          const LlaPositionDegM& origin_lla,
                          EnuVelocityMps* enu_velocity) {
  if (enu_velocity == nullptr || !IsFinite(ecef_velocity) || !IsValid(origin_lla)) {
    return false;
  }

  const double lat_rad = oneq::common::numerics::DegToRad(origin_lla.latitude_deg);
  const double lon_rad = oneq::common::numerics::DegToRad(origin_lla.longitude_deg);
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

bool TryEnuToEcefVelocity(const EnuVelocityMps& enu_velocity,
                          const LlaPositionDegM& origin_lla,
                          EcefVelocityMps* ecef_velocity) {
  if (ecef_velocity == nullptr || !IsFinite(enu_velocity) || !IsValid(origin_lla)) {
    return false;
  }

  const double lat_rad = oneq::common::numerics::DegToRad(origin_lla.latitude_deg);
  const double lon_rad = oneq::common::numerics::DegToRad(origin_lla.longitude_deg);
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);

  ecef_velocity->x_mps = -sin_lon * enu_velocity.east_mps -
                         sin_lat * cos_lon * enu_velocity.north_mps +
                         cos_lat * cos_lon * enu_velocity.up_mps;
  ecef_velocity->y_mps = cos_lon * enu_velocity.east_mps -
                         sin_lat * sin_lon * enu_velocity.north_mps +
                         cos_lat * sin_lon * enu_velocity.up_mps;
  ecef_velocity->z_mps =
      cos_lat * enu_velocity.north_mps + sin_lat * enu_velocity.up_mps;
  return IsFinite(*ecef_velocity);
}

bool TryMakeEcefVelocityFromHeading(double heading_deg, double speed_mps,
                                    const LlaPositionDegM& origin_lla,
                                    EcefVelocityMps* ecef_velocity) {
  if (ecef_velocity == nullptr || !IsValid(origin_lla) ||
      !oneq::common::validation::IsFinite(heading_deg) ||
      !oneq::common::validation::IsFinite(speed_mps) || speed_mps < 0.0) {
    return false;
  }
  // 平台运动学约定：水平速度按航向（北偏东）分解，up=0。
  const double heading_rad = oneq::common::numerics::DegToRad(heading_deg);
  EnuVelocityMps enu_velocity;
  enu_velocity.east_mps = speed_mps * std::sin(heading_rad);
  enu_velocity.north_mps = speed_mps * std::cos(heading_rad);
  enu_velocity.up_mps = 0.0;
  return TryEnuToEcefVelocity(enu_velocity, origin_lla, ecef_velocity);
}

}  // namespace coordinate
}  // namespace oneq
