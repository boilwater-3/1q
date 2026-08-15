#include "1q/coordinate/inertial_transform.h"

#include <cmath>

#include "common/numerics/Constants.h"

namespace oneq {
namespace coordinate {
namespace {

// IAU 1982 近似 GMST 公式（Vallado 式 3-47）系数。
constexpr double kGmstCoeffA0 = 280.46061837;       // deg
constexpr double kGmstCoeffA1 = 360.98564736629;    // deg / day
constexpr double kGmstCoeffA2 = 0.000387933;        // deg / century²
constexpr double kGmstCoeffA3 = 1.0 / 38710000.0;   // deg / century³
constexpr double kJ2000JulianDay = 2451545.0;
constexpr double kJulianCenturyDays = 36525.0;

bool IsFiniteVec(double x, double y, double z) { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }

// 绕 z 轴旋转矩阵 R3(θ)（列主序手写应用）：r' = R3(θ)·r。
// R3(θ) = [[cosθ, −sinθ, 0], [sinθ, cosθ, 0], [0, 0, 1]]。
void RotateZ(double theta_rad, double x, double y, double z, double* out_x, double* out_y,
             double* out_z) {
  const double cos_theta = std::cos(theta_rad);
  const double sin_theta = std::sin(theta_rad);
  *out_x = cos_theta * x - sin_theta * y;
  *out_y = sin_theta * x + cos_theta * y;
  *out_z = z;
}

double PositiveModulo(double value, double period) {
  const double result = std::fmod(value, period);
  return result < 0.0 ? result + period : result;
}

}  // namespace

bool TryComputeGmstRad(double utc_julian_day, double* gmst_rad) {
  if (gmst_rad == nullptr || !std::isfinite(utc_julian_day) || utc_julian_day <= 0.0) {
    return false;
  }
  const double d = utc_julian_day - kJ2000JulianDay;
  const double t = d / kJulianCenturyDays;
  const double gmst_deg =
      kGmstCoeffA0 + kGmstCoeffA1 * d + kGmstCoeffA2 * t * t - kGmstCoeffA3 * t * t * t;
  *gmst_rad = oneq::common::numerics::DegToRad(PositiveModulo(gmst_deg, 360.0));
  return true;
}

bool TryEcefToEci(const EcefPositionM& ecef, double gmst_rad, EciPositionM* eci) {
  if (eci == nullptr || !std::isfinite(gmst_rad) || !IsFiniteVec(ecef.x_m, ecef.y_m, ecef.z_m)) {
    return false;
  }
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  RotateZ(gmst_rad, ecef.x_m, ecef.y_m, ecef.z_m, &x, &y, &z);
  *eci = EciPositionM(x, y, z);
  return true;
}

bool TryEciToEcef(const EciPositionM& eci, double gmst_rad, EcefPositionM* ecef) {
  if (ecef == nullptr || !std::isfinite(gmst_rad) || !IsFiniteVec(eci.x_m, eci.y_m, eci.z_m)) {
    return false;
  }
  // 逆旋转 R3(−θ)。
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  RotateZ(-gmst_rad, eci.x_m, eci.y_m, eci.z_m, &x, &y, &z);
  *ecef = EcefPositionM(x, y, z);
  return true;
}

bool TryEcefVelocityToEci(const EcefPositionM& ecef_position,
                          const EcefVelocityMps& ecef_velocity, double gmst_rad,
                          EciVelocityMps* eci_velocity) {
  if (eci_velocity == nullptr || !std::isfinite(gmst_rad) ||
      !IsFiniteVec(ecef_position.x_m, ecef_position.y_m, ecef_position.z_m) ||
      !IsFiniteVec(ecef_velocity.x_mps, ecef_velocity.y_mps, ecef_velocity.z_mps)) {
    return false;
  }
  EciPositionM eci_position;
  if (!TryEcefToEci(ecef_position, gmst_rad, &eci_position)) {
    return false;
  }
  // v_ECI = R3(θ)·v_ECEF + ω_e × r_ECI；ω_e = (0, 0, ω_e) → ω×r = (−ω·y, ω·x, 0)。
  double rotated_x = 0.0;
  double rotated_y = 0.0;
  double rotated_z = 0.0;
  RotateZ(gmst_rad, ecef_velocity.x_mps, ecef_velocity.y_mps, ecef_velocity.z_mps, &rotated_x,
          &rotated_y, &rotated_z);
  const double omega = kEarthRotationRateRadPerSec;
  *eci_velocity = EciVelocityMps(rotated_x - omega * eci_position.y_m,
                                 rotated_y + omega * eci_position.x_m, rotated_z);
  return true;
}

}  // namespace coordinate
}  // namespace oneq