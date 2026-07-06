#include "sbirs_sensor/foundation/SbirsGeometry.h"

#include <algorithm>
#include <cmath>

namespace sbirs_sensor {
namespace foundation {
namespace {

const double kPi = 3.14159265358979323846;
const double kRadToDeg = 180.0 / kPi;
const double kDegToRad = kPi / 180.0;

double Clamp(double value, double low, double high) { return std::max(low, std::min(high, value)); }

}  // namespace

double Dot(const session::SbirsVector3M& lhs, const session::SbirsVector3M& rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

double Norm(const session::SbirsVector3M& value) { return std::sqrt(Dot(value, value)); }

session::SbirsVector3M Subtract(const session::SbirsVector3M& lhs,
                                const session::SbirsVector3M& rhs) {
  session::SbirsVector3M result;
  result.x = lhs.x - rhs.x;
  result.y = lhs.y - rhs.y;
  result.z = lhs.z - rhs.z;
  return result;
}

session::SbirsVector3M Unit(const session::SbirsVector3M& value) {
  const double norm = Norm(value);
  if (norm <= 0.0) {
    return session::SbirsVector3M{};
  }
  session::SbirsVector3M result;
  result.x = value.x / norm;
  result.y = value.y / norm;
  result.z = value.z / norm;
  return result;
}

float ComputeAzimuthDeg(const session::SbirsVector3M& los) {
  return static_cast<float>(std::atan2(los.y, los.x) * kRadToDeg);
}

float ComputeElevationDeg(const session::SbirsVector3M& los) {
  const double range = Norm(los);
  if (range <= 0.0) {
    return 0.0f;
  }
  return static_cast<float>(std::asin(Clamp(los.z / range, -1.0, 1.0)) * kRadToDeg);
}

float AngularSeparationDeg(float az_a_deg, float el_a_deg, float az_b_deg, float el_b_deg) {
  const double az_a = az_a_deg * kDegToRad;
  const double el_a = el_a_deg * kDegToRad;
  const double az_b = az_b_deg * kDegToRad;
  const double el_b = el_b_deg * kDegToRad;
  const double dot =
      std::sin(el_a) * std::sin(el_b) + std::cos(el_a) * std::cos(el_b) * std::cos(az_a - az_b);
  return static_cast<float>(std::acos(Clamp(dot, -1.0, 1.0)) * kRadToDeg);
}

bool IsEarthOcculted(const session::SbirsVector3M& satellite_position_ecef_m,
                     const session::SbirsVector3M& target_position_ecef_m, double earth_radius_m) {
  const session::SbirsVector3M los = Subtract(target_position_ecef_m, satellite_position_ecef_m);
  const double range = Norm(los);
  if (range <= 0.0 || earth_radius_m <= 0.0) {
    return false;
  }
  const session::SbirsVector3M u = Unit(los);
  const double s_closest = -Dot(satellite_position_ecef_m, u);
  if (s_closest <= 0.0 || s_closest >= range) {
    return false;
  }
  const double sat_norm_sq = Dot(satellite_position_ecef_m, satellite_position_ecef_m);
  const double closest_sq = sat_norm_sq - s_closest * s_closest;
  return closest_sq <= earth_radius_m * earth_radius_m;
}

}  // namespace foundation
}  // namespace sbirs_sensor
