#include "1q/coordinate/position_transform.h"

#include <cmath>
#include "common/validation/ValidationUtils.h"
#include "common/numerics/Constants.h"

namespace oneq {
namespace coordinate {

namespace {

constexpr double kWgs84SemiMajorAxisM = 6378137.0;
constexpr double kWgs84Flattening = 1.0 / 298.257223563;
constexpr double kWgs84SemiMinorAxisM = kWgs84SemiMajorAxisM * (1.0 - kWgs84Flattening);
constexpr double kWgs84EccentricitySquared = kWgs84Flattening * (2.0 - kWgs84Flattening);
constexpr double kWgs84SecondEccentricitySquared =
    (kWgs84SemiMajorAxisM * kWgs84SemiMajorAxisM -
     kWgs84SemiMinorAxisM * kWgs84SemiMinorAxisM) /
    (kWgs84SemiMinorAxisM * kWgs84SemiMinorAxisM);
constexpr double kNormFloor = 1.0e-9;


}  // namespace

bool IsValid(const LlaPositionDegM& lla) {
  if (!oneq::common::validation::IsFinite(lla.latitude_deg) || !oneq::common::validation::IsFinite(lla.longitude_deg) ||
      !oneq::common::validation::IsFinite(lla.altitude_m)) {
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

bool IsFinite(const EcefPositionM& ecef) {
  return oneq::common::validation::IsFinite(ecef.x_m) && oneq::common::validation::IsFinite(ecef.y_m) && oneq::common::validation::IsFinite(ecef.z_m);
}

bool IsFinite(const EnuPositionM& enu) {
  return oneq::common::validation::IsFinite(enu.east_m) && oneq::common::validation::IsFinite(enu.north_m) &&
         oneq::common::validation::IsFinite(enu.up_m);
}

bool IsFinite(const NedPositionM& ned) {
  return oneq::common::validation::IsFinite(ned.north_m) && oneq::common::validation::IsFinite(ned.east_m) &&
         oneq::common::validation::IsFinite(ned.down_m);
}

bool IsFinite(const NuePositionM& nue) {
  return oneq::common::validation::IsFinite(nue.north_m) && oneq::common::validation::IsFinite(nue.up_m) &&
         oneq::common::validation::IsFinite(nue.east_m);
}

bool TryLlaToEcef(const LlaPositionDegM& lla, EcefPositionM* ecef) {
  if (ecef == nullptr || !IsValid(lla)) {
    return false;
  }

  const double lat_rad = oneq::common::numerics::DegToRad(lla.latitude_deg);
  const double lon_rad = oneq::common::numerics::DegToRad(lla.longitude_deg);
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

bool TryEcefToLla(const EcefPositionM& ecef, LlaPositionDegM* lla) {
  if (lla == nullptr || !IsFinite(ecef)) {
    return false;
  }

  const double p = std::sqrt(ecef.x_m * ecef.x_m + ecef.y_m * ecef.y_m);
  const double norm =
      std::sqrt(ecef.x_m * ecef.x_m + ecef.y_m * ecef.y_m + ecef.z_m * ecef.z_m);
  if (norm <= kNormFloor) {
    return false;
  }

  const double lon_rad = std::atan2(ecef.y_m, ecef.x_m);
  const double theta =
      std::atan2(ecef.z_m * kWgs84SemiMajorAxisM, p * kWgs84SemiMinorAxisM);
  const double sin_theta = std::sin(theta);
  const double cos_theta = std::cos(theta);
  const double lat_rad = std::atan2(
      ecef.z_m + kWgs84SecondEccentricitySquared * kWgs84SemiMinorAxisM * sin_theta *
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
    alt_m =
        ecef.z_m / sin_lat - prime_vertical_radius * (1.0 - kWgs84EccentricitySquared);
  }

  lla->latitude_deg = oneq::common::numerics::RadToDeg(lat_rad);
  lla->longitude_deg = oneq::common::numerics::RadToDeg(lon_rad);
  lla->altitude_m = alt_m;
  return IsValid(*lla);
}

bool TryEcefToEnu(const EcefPositionM& ecef,
                  const LlaPositionDegM& origin_lla,
                  EnuPositionM* enu) {
  if (enu == nullptr || !IsFinite(ecef) || !IsValid(origin_lla)) {
    return false;
  }

  EcefPositionM origin_ecef;
  if (!TryLlaToEcef(origin_lla, &origin_ecef)) {
    return false;
  }

  const double lat_rad = oneq::common::numerics::DegToRad(origin_lla.latitude_deg);
  const double lon_rad = oneq::common::numerics::DegToRad(origin_lla.longitude_deg);
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);

  const double dx = ecef.x_m - origin_ecef.x_m;
  const double dy = ecef.y_m - origin_ecef.y_m;
  const double dz = ecef.z_m - origin_ecef.z_m;

  enu->east_m = -sin_lon * dx + cos_lon * dy;
  enu->north_m = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz;
  enu->up_m = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz;
  return IsFinite(*enu);
}

bool TryEcefToNed(const EcefPositionM& ecef,
                  const LlaPositionDegM& origin_lla,
                  NedPositionM* ned) {
  if (ned == nullptr) {
    return false;
  }
  EnuPositionM enu;
  if (!TryEcefToEnu(ecef, origin_lla, &enu)) {
    return false;
  }
  *ned = ToNed(enu);
  return true;
}

bool TryEcefToNue(const EcefPositionM& ecef,
                  const LlaPositionDegM& origin_lla,
                  NuePositionM* nue) {
  if (nue == nullptr) {
    return false;
  }
  EnuPositionM enu;
  if (!TryEcefToEnu(ecef, origin_lla, &enu)) {
    return false;
  }
  *nue = ToNue(enu);
  return true;
}

bool TryLlaToEnu(const LlaPositionDegM& lla,
                 const LlaPositionDegM& origin_lla,
                 EnuPositionM* enu) {
  if (enu == nullptr || !IsValid(lla)) {
    return false;
  }
  EcefPositionM ecef;
  if (!TryLlaToEcef(lla, &ecef)) {
    return false;
  }
  return TryEcefToEnu(ecef, origin_lla, enu);
}

bool TryLlaToNed(const LlaPositionDegM& lla,
                 const LlaPositionDegM& origin_lla,
                 NedPositionM* ned) {
  if (ned == nullptr) {
    return false;
  }
  EnuPositionM enu;
  if (!TryLlaToEnu(lla, origin_lla, &enu)) {
    return false;
  }
  *ned = ToNed(enu);
  return true;
}

bool TryLlaToNue(const LlaPositionDegM& lla,
                 const LlaPositionDegM& origin_lla,
                 NuePositionM* nue) {
  if (nue == nullptr) {
    return false;
  }
  EnuPositionM enu;
  if (!TryLlaToEnu(lla, origin_lla, &enu)) {
    return false;
  }
  *nue = ToNue(enu);
  return true;
}

NedPositionM ToNed(const EnuPositionM& enu) {
  NedPositionM ned;
  ned.north_m = enu.north_m;
  ned.east_m = enu.east_m;
  ned.down_m = -enu.up_m;
  return ned;
}

EnuPositionM ToEnu(const NedPositionM& ned) {
  EnuPositionM enu;
  enu.east_m = ned.east_m;
  enu.north_m = ned.north_m;
  enu.up_m = -ned.down_m;
  return enu;
}

NuePositionM ToNue(const EnuPositionM& enu) {
  NuePositionM nue;
  nue.north_m = enu.north_m;
  nue.up_m = enu.up_m;
  nue.east_m = enu.east_m;
  return nue;
}

EnuPositionM ToEnu(const NuePositionM& nue) {
  EnuPositionM enu;
  enu.east_m = nue.east_m;
  enu.north_m = nue.north_m;
  enu.up_m = nue.up_m;
  return enu;
}

NuePositionM ToNue(const NedPositionM& ned) {
  NuePositionM nue;
  nue.north_m = ned.north_m;
  nue.up_m = -ned.down_m;
  nue.east_m = ned.east_m;
  return nue;
}

NedPositionM ToNed(const NuePositionM& nue) {
  NedPositionM ned;
  ned.north_m = nue.north_m;
  ned.east_m = nue.east_m;
  ned.down_m = -nue.up_m;
  return ned;
}

bool TryEnuToEcef(const EnuPositionM& enu,
                  const LlaPositionDegM& origin_lla,
                  EcefPositionM* ecef) {
  if (ecef == nullptr || !IsFinite(enu) || !IsValid(origin_lla)) {
    return false;
  }

  EcefPositionM origin_ecef;
  if (!TryLlaToEcef(origin_lla, &origin_ecef)) {
    return false;
  }

  const double lat_rad = oneq::common::numerics::DegToRad(origin_lla.latitude_deg);
  const double lon_rad = oneq::common::numerics::DegToRad(origin_lla.longitude_deg);
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);

  ecef->x_m =
      origin_ecef.x_m - sin_lon * enu.east_m - sin_lat * cos_lon * enu.north_m +
      cos_lat * cos_lon * enu.up_m;
  ecef->y_m =
      origin_ecef.y_m + cos_lon * enu.east_m - sin_lat * sin_lon * enu.north_m +
      cos_lat * sin_lon * enu.up_m;
  ecef->z_m = origin_ecef.z_m + cos_lat * enu.north_m + sin_lat * enu.up_m;
  return IsFinite(*ecef);
}

bool TryBearingRangeToEnuOffset(double azimuth_deg, double range_m,
                                EnuPositionM* enu_offset) {
  if (enu_offset == nullptr || !oneq::common::validation::IsFinite(azimuth_deg) ||
      !oneq::common::validation::IsFinite(range_m) || range_m < 0.0) {
    return false;
  }
  // 纯几何约定：方位（北偏东）与水平距离 → ENU 水平偏移（up=0）。
  const double azimuth_rad = oneq::common::numerics::DegToRad(azimuth_deg);
  enu_offset->east_m = range_m * std::sin(azimuth_rad);
  enu_offset->north_m = range_m * std::cos(azimuth_rad);
  enu_offset->up_m = 0.0;
  return true;
}

bool TryEnuToEcefDirection(const Vector3d& enu_dir,
                           const LlaPositionDegM& origin_lla,
                           Vector3d* ecef_dir) {
  if (ecef_dir == nullptr || !IsValid(origin_lla)) {
    return false;
  }

  const double lat_rad = oneq::common::numerics::DegToRad(origin_lla.latitude_deg);
  const double lon_rad = oneq::common::numerics::DegToRad(origin_lla.longitude_deg);
  const double sin_lat = std::sin(lat_rad);
  const double cos_lat = std::cos(lat_rad);
  const double sin_lon = std::sin(lon_rad);
  const double cos_lon = std::cos(lon_rad);

  ecef_dir->x = -sin_lon * enu_dir.x - sin_lat * cos_lon * enu_dir.y +
                cos_lat * cos_lon * enu_dir.z;
  ecef_dir->y = cos_lon * enu_dir.x - sin_lat * sin_lon * enu_dir.y +
                cos_lat * sin_lon * enu_dir.z;
  ecef_dir->z = cos_lat * enu_dir.y + sin_lat * enu_dir.z;
  const double norm = std::sqrt(ecef_dir->x * ecef_dir->x + ecef_dir->y * ecef_dir->y +
                                ecef_dir->z * ecef_dir->z);
  if (norm <= kNormFloor) {
    return false;
  }
  ecef_dir->x /= norm;
  ecef_dir->y /= norm;
  ecef_dir->z /= norm;
  return oneq::common::validation::IsFinite(ecef_dir->x) && oneq::common::validation::IsFinite(ecef_dir->y) &&
         oneq::common::validation::IsFinite(ecef_dir->z);
}

}  // namespace coordinate
}  // namespace oneq
