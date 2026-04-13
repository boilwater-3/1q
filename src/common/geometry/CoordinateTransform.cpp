#include "1q/foundation/coordinate_transform.h"

#include <cmath>

namespace oneq {
namespace foundation {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kWgs84SemiMajorAxisM = 6378137.0;
constexpr double kWgs84Flattening = 1.0 / 298.257223563;
constexpr double kWgs84SemiMinorAxisM = kWgs84SemiMajorAxisM * (1.0 - kWgs84Flattening);
constexpr double kWgs84EccentricitySquared = kWgs84Flattening * (2.0 - kWgs84Flattening);
constexpr double kWgs84SecondEccentricitySquared =
    (kWgs84SemiMajorAxisM * kWgs84SemiMajorAxisM - kWgs84SemiMinorAxisM * kWgs84SemiMinorAxisM) /
    (kWgs84SemiMinorAxisM * kWgs84SemiMinorAxisM);
constexpr double kNormFloor = 1.0e-9;

inline bool IsFinite(double value) { return std::isfinite(value) != 0; }

inline double DegToRad(double deg) { return deg * kPi / 180.0; }
inline double RadToDeg(double rad) { return rad * 180.0 / kPi; }

}  // namespace

bool IsValidLla(const LlaCoordinateDegM& lla) {
  if (!IsFinite(lla.latitude_deg) || !IsFinite(lla.longitude_deg) || !IsFinite(lla.altitude_m)) {
    return false;
  }
  if (lla.latitude_deg < -90.0 || lla.latitude_deg > 90.0) {
    return false;
  }
  if (lla.longitude_deg < -180.0 || lla.longitude_deg > 180.0) {
    return false;
  }
  return true;
}

bool TryLlaToEcef(const LlaCoordinateDegM& lla, EcefCoordinateM* ecef) {
  if (ecef == nullptr || !IsValidLla(lla)) {
    return false;
  }

  const double lat_rad = DegToRad(lla.latitude_deg);
  const double lon_rad = DegToRad(lla.longitude_deg);
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);
  const double prime_vertical_radius =
      kWgs84SemiMajorAxisM / std::sqrt(1.0 - kWgs84EccentricitySquared * sin_lat * sin_lat);

  ecef->x_m = (prime_vertical_radius + lla.altitude_m) * cos_lat * cos_lon;
  ecef->y_m = (prime_vertical_radius + lla.altitude_m) * cos_lat * sin_lon;
  ecef->z_m =
      (prime_vertical_radius * (1.0 - kWgs84EccentricitySquared) + lla.altitude_m) * sin_lat;
  return true;
}

bool TryEcefToLla(const EcefCoordinateM& ecef, LlaCoordinateDegM* lla) {
  if (lla == nullptr || !IsFinite(ecef.x_m) || !IsFinite(ecef.y_m) || !IsFinite(ecef.z_m)) {
    return false;
  }

  const double p = std::sqrt(ecef.x_m * ecef.x_m + ecef.y_m * ecef.y_m);
  const double norm = std::sqrt(ecef.x_m * ecef.x_m + ecef.y_m * ecef.y_m + ecef.z_m * ecef.z_m);
  if (norm <= kNormFloor) {
    return false;
  }

  const double lon_rad = std::atan2(ecef.y_m, ecef.x_m);
  const double theta = std::atan2(ecef.z_m * kWgs84SemiMajorAxisM, p * kWgs84SemiMinorAxisM);
  const double sin_theta = std::sin(theta);
  const double cos_theta = std::cos(theta);
  const double lat_rad =
      std::atan2(ecef.z_m +
                     kWgs84SecondEccentricitySquared * kWgs84SemiMinorAxisM * sin_theta *
                         sin_theta * sin_theta,
                 p - kWgs84EccentricitySquared * kWgs84SemiMajorAxisM * cos_theta * cos_theta *
                         cos_theta);

  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double prime_vertical_radius =
      kWgs84SemiMajorAxisM / std::sqrt(1.0 - kWgs84EccentricitySquared * sin_lat * sin_lat);

  double alt_m = 0.0;
  if (std::abs(cos_lat) > kNormFloor) {
    alt_m = p / cos_lat - prime_vertical_radius;
  } else {
    alt_m = ecef.z_m / sin_lat - prime_vertical_radius * (1.0 - kWgs84EccentricitySquared);
  }

  lla->latitude_deg = RadToDeg(lat_rad);
  lla->longitude_deg = RadToDeg(lon_rad);
  lla->altitude_m = alt_m;
  return IsValidLla(*lla);
}

bool TryEcefToEnu(const EcefCoordinateM& ecef, const LlaCoordinateDegM& origin_lla,
                  EnuCoordinateM* enu) {
  if (enu == nullptr || !IsValidLla(origin_lla)) {
    return false;
  }

  EcefCoordinateM origin_ecef;
  if (!TryLlaToEcef(origin_lla, &origin_ecef)) {
    return false;
  }

  const double lat_rad = DegToRad(origin_lla.latitude_deg);
  const double lon_rad = DegToRad(origin_lla.longitude_deg);
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);

  const double dx = ecef.x_m - origin_ecef.x_m;
  const double dy = ecef.y_m - origin_ecef.y_m;
  const double dz = ecef.z_m - origin_ecef.z_m;

  enu->x_m = -sin_lon * dx + cos_lon * dy;
  enu->y_m = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
  enu->z_m = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;
  return IsFinite(enu->x_m) && IsFinite(enu->y_m) && IsFinite(enu->z_m);
}

bool TryLlaToEnu(const LlaCoordinateDegM& lla, const LlaCoordinateDegM& origin_lla,
                 EnuCoordinateM* enu) {
  if (enu == nullptr || !IsValidLla(lla)) {
    return false;
  }

  EcefCoordinateM ecef;
  if (!TryLlaToEcef(lla, &ecef)) {
    return false;
  }
  return TryEcefToEnu(ecef, origin_lla, enu);
}

bool TryEcefToNue(const EcefCoordinateM& ecef, const LlaCoordinateDegM& origin_lla,
                  NueCoordinateM* nue) {
  if (nue == nullptr) {
    return false;
  }
  EnuCoordinateM enu;
  if (!TryEcefToEnu(ecef, origin_lla, &enu)) {
    return false;
  }
  nue->x_m = enu.y_m;
  nue->y_m = enu.z_m;
  nue->z_m = enu.x_m;
  return true;
}

bool TryLlaToNue(const LlaCoordinateDegM& lla, const LlaCoordinateDegM& origin_lla,
                 NueCoordinateM* nue) {
  if (nue == nullptr || !IsValidLla(lla)) {
    return false;
  }

  EcefCoordinateM ecef;
  if (!TryLlaToEcef(lla, &ecef)) {
    return false;
  }
  return TryEcefToNue(ecef, origin_lla, nue);
}

Vector3f ToVector3f(const EnuCoordinateM& enu) {
  Vector3f vector;
  vector.x = static_cast<float>(enu.x_m);
  vector.y = static_cast<float>(enu.y_m);
  vector.z = static_cast<float>(enu.z_m);
  return vector;
}

Vector3f ToVector3f(const NueCoordinateM& nue) {
  Vector3f vector;
  vector.x = static_cast<float>(nue.x_m);
  vector.y = static_cast<float>(nue.y_m);
  vector.z = static_cast<float>(nue.z_m);
  return vector;
}

}  // namespace foundation
}  // namespace oneq
