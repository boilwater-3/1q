/**
 * @file PolarizationAcceptanceS.cpp
 * @brief 验收旁路 Sinclair S：功率迹 / |det| / 去极化 / Graves ψτ（未进识别）。
 */

#include "remote_identification_radar/runtime/PolarizationAcceptanceS.h"

#include <cmath>
#include <complex>
#include <limits>

namespace remote_identification_radar {
namespace runtime {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr double kOffDiagonalFloor = 1.0e-18;

bool IsFiniteLook(float az_deg, float el_deg) {
  return std::isfinite(az_deg) && std::isfinite(el_deg);
}

double DbsmToLinear(float dbsm) { return std::pow(10.0, static_cast<double>(dbsm) / 10.0); }

bool TryNearestSample(const std::vector<session::RirPolarizationRcsSample>& samples, float az_deg,
                      float el_deg, session::RirPolarizationRcsSample* nearest) {
  if (nearest == nullptr || samples.empty()) {
    return false;
  }
  float best_d = std::numeric_limits<float>::infinity();
  bool found = false;
  for (std::size_t i = 0U; i < samples.size(); ++i) {
    const float d_az = samples[i].aspect_az_deg - az_deg;
    const float d_el = samples[i].aspect_el_deg - el_deg;
    const float distance = std::sqrt(d_az * d_az + d_el * d_el);
    if (distance < best_d) {
      best_d = distance;
      *nearest = samples[i];
      found = true;
    }
  }
  return found;
}

bool SampleReadyForS(const session::RirPolarizationRcsSample& sample) {
  return sample.has_cross_pol && sample.has_phase_vv && std::isfinite(sample.channel_1_rcs_dbsm) &&
         std::isfinite(sample.channel_2_rcs_dbsm) && std::isfinite(sample.cross_rcs_dbsm) &&
         std::isfinite(sample.phase_vv_rel_hh_deg);
}

}  // namespace

bool TryResolvePolarizationAcceptanceS(
    const std::vector<session::RirPolarizationRcsSample>& samples, float look_az_deg,
    float look_el_deg, PolarizationAcceptanceSResult* result) {
  if (result == nullptr || !IsFiniteLook(look_az_deg, look_el_deg)) {
    return false;
  }
  session::RirPolarizationRcsSample nearest;
  if (!TryNearestSample(samples, look_az_deg, look_el_deg, &nearest) || !SampleReadyForS(nearest)) {
    return false;
  }

  const double sigma_hh = DbsmToLinear(nearest.channel_1_rcs_dbsm);
  const double sigma_vv = DbsmToLinear(nearest.channel_2_rcs_dbsm);
  const double sigma_hv = DbsmToLinear(nearest.cross_rcs_dbsm);
  if (!std::isfinite(sigma_hh) || !std::isfinite(sigma_vv) || !std::isfinite(sigma_hv) ||
      sigma_hh < 0.0 || sigma_vv < 0.0 || sigma_hv < 0.0) {
    return false;
  }

  const double span = sigma_hh + sigma_vv + 2.0 * sigma_hv;
  if (!(span > 0.0) || !std::isfinite(span)) {
    return false;
  }

  using Complex = std::complex<double>;
  const double phase_rad = static_cast<double>(nearest.phase_vv_rel_hh_deg) * kPi / 180.0;
  const Complex shh(std::sqrt(sigma_hh), 0.0);
  const Complex svv = std::sqrt(sigma_vv) * Complex(std::cos(phase_rad), std::sin(phase_rad));
  const Complex shv(std::sqrt(sigma_hv), 0.0);
  const Complex svh = shv;
  const Complex det = shh * svv - shv * svh;
  const double abs_det = std::abs(det);
  if (!std::isfinite(abs_det)) {
    return false;
  }

  // Graves G = SᴴS（2×2 Hermitian）。
  const double a = std::norm(shh) + std::norm(svh);
  const double b = std::norm(shv) + std::norm(svv);
  const Complex c = std::conj(shh) * shv + std::conj(svh) * svv;
  const double disc = std::sqrt((a - b) * (a - b) + 4.0 * std::norm(c));
  const double lambda = 0.5 * (a + b + disc);

  double psi_deg = 0.0;
  double tau_deg = 0.0;
  if (std::norm(c) <= kOffDiagonalFloor * kOffDiagonalFloor) {
    if (b > a) {
      psi_deg = 90.0;
    }
  } else {
    const Complex rho = (lambda - a) / c;
    const double rho_abs2 = std::norm(rho);
    const double psi_rad = 0.5 * std::atan2(2.0 * rho.real(), 1.0 - rho_abs2);
    double tau_arg = 2.0 * rho.imag() / (1.0 + rho_abs2);
    if (tau_arg > 1.0) {
      tau_arg = 1.0;
    } else if (tau_arg < -1.0) {
      tau_arg = -1.0;
    }
    const double tau_rad = 0.5 * std::asin(tau_arg);
    psi_deg = psi_rad * kRadToDeg;
    tau_deg = tau_rad * kRadToDeg;
  }
  if (!std::isfinite(psi_deg) || !std::isfinite(tau_deg)) {
    return false;
  }

  result->span = span;
  result->abs_det = abs_det;
  result->depolarization = 2.0 * sigma_hv / span;
  result->psi_deg = psi_deg;
  result->tau_deg = tau_deg;
  return true;
}

}  // namespace runtime
}  // namespace remote_identification_radar
