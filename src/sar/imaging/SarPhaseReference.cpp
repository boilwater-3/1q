#include "sar/imaging/SarPhaseReference.h"

#include <algorithm>
#include <cmath>
#include <complex>

namespace sar {
namespace imaging {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kSpeedOfLightMps = 299792458.0;

bool IsValidMatrix(const signal::ComplexMatrix& image) {
  return image.rows > 0U && image.cols > 0U && image.values.size() == image.rows * image.cols;
}

bool IsValidReferenceConfig(const PhaseReferenceConfig& config) {
  return config.carrier_frequency_hz > 0.0 && std::isfinite(config.carrier_frequency_hz) &&
         config.prf_hz > 0.0 && std::isfinite(config.prf_hz) &&
         config.platform_velocity_mps > 0.0 && std::isfinite(config.platform_velocity_mps) &&
         config.range_bin_spacing_m > 0.0 && std::isfinite(config.range_bin_spacing_m);
}

}  // namespace

bool NeedsPhaseReference(PhaseReferenceMode mode, bool* needs_reference) {
  if (needs_reference == nullptr) {
    return false;
  }
  switch (mode) {
    case PhaseReferenceMode::kNative:
      *needs_reference = false;
      return true;
    case PhaseReferenceMode::kCenterBroadside:
      *needs_reference = true;
      return true;
  }
  return false;
}

bool ApplyBroadsideCenterPhaseReference(const PhaseReferenceConfig& config,
                                        signal::ComplexMatrix* image,
                                        PhaseReferenceDiagnostics* diagnostics) {
  bool needs_reference = false;
  if (image == nullptr || diagnostics == nullptr || !IsValidMatrix(*image) ||
      !NeedsPhaseReference(config.mode, &needs_reference)) {
    return false;
  }

  *diagnostics = PhaseReferenceDiagnostics{};
  diagnostics->mode = config.mode;
  if (!needs_reference) {
    return true;
  }
  if (!IsValidReferenceConfig(config)) {
    return false;
  }

  const double wavelength_m = kSpeedOfLightMps / config.carrier_frequency_hz;
  const double center_row = 0.5 * static_cast<double>(image->rows - 1U);
  bool first_phase = true;
  for (std::size_t row = 0U; row < image->rows; ++row) {
    const double slow_time_s = (static_cast<double>(row) - center_row) / config.prf_hz;
    const double x_m = config.platform_velocity_mps * slow_time_s;
    for (std::size_t col = 0U; col < image->cols; ++col) {
      const double range_m =
          std::max(static_cast<double>(col) * config.range_bin_spacing_m,
                   0.5 * config.range_bin_spacing_m);
      const double slant_m = std::sqrt(range_m * range_m + x_m * x_m);
      const double phase = 4.0 * kPi * slant_m / wavelength_m;
      (*image)(row, col) *= signal::ComplexSample(std::cos(phase), std::sin(phase));
      if (first_phase) {
        diagnostics->min_phase_rad = phase;
        diagnostics->max_phase_rad = phase;
        first_phase = false;
      } else {
        diagnostics->min_phase_rad = std::min(diagnostics->min_phase_rad, phase);
        diagnostics->max_phase_rad = std::max(diagnostics->max_phase_rad, phase);
      }
    }
  }
  diagnostics->applied = true;
  return true;
}

bool EstimateGlobalPhaseOffset(const signal::ComplexMatrix& reference,
                               const signal::ComplexMatrix& candidate,
                               double* phase_offset_rad) {
  if (phase_offset_rad == nullptr || !IsValidMatrix(reference) || !IsValidMatrix(candidate) ||
      reference.rows != candidate.rows || reference.cols != candidate.cols) {
    return false;
  }

  signal::ComplexSample cross(0.0, 0.0);
  double reference_power = 0.0;
  double candidate_power = 0.0;
  for (std::size_t index = 0U; index < reference.values.size(); ++index) {
    cross += reference.values[index] * std::conj(candidate.values[index]);
    reference_power += std::norm(reference.values[index]);
    candidate_power += std::norm(candidate.values[index]);
  }
  if (reference_power <= 0.0 || candidate_power <= 0.0) {
    return false;
  }

  *phase_offset_rad = std::arg(cross);
  return true;
}

bool ApplyGlobalPhaseOffset(double phase_offset_rad, signal::ComplexMatrix* image) {
  if (image == nullptr || !IsValidMatrix(*image) || !std::isfinite(phase_offset_rad)) {
    return false;
  }
  const signal::ComplexSample rotation(std::cos(phase_offset_rad), std::sin(phase_offset_rad));
  for (signal::ComplexSample& sample : image->values) {
    sample *= rotation;
  }
  return true;
}

}  // namespace imaging
}  // namespace sar
