#include "sar/imaging/SarAutofocusPhaseTruth.h"

#include <algorithm>
#include <cmath>

namespace sar {
namespace imaging {

namespace {

bool IsFinite(double value) {
  return std::isfinite(value);
}

}  // namespace

bool EvaluateAutofocusPhaseTruth(const AutofocusPhaseTruthConfig& config,
                                 AutofocusPhaseTruthDiagnostics* diagnostics) {
  if (diagnostics == nullptr) {
    return false;
  }
  *diagnostics = AutofocusPhaseTruthDiagnostics{};
  if (config.sample_count < 3U || !IsFinite(config.constant_rad) ||
      !IsFinite(config.linear_rad) || !IsFinite(config.quadratic_rad) ||
      !IsFinite(config.cubic_rad)) {
    return false;
  }

  diagnostics->sample_count = config.sample_count;
  diagnostics->normalized_aperture_coordinates.reserve(config.sample_count);
  diagnostics->raw_phase_error_rad.reserve(config.sample_count);
  double sum_x = 0.0;
  double sum_xx = 0.0;
  double sum_phase = 0.0;
  double sum_x_phase = 0.0;
  for (std::size_t index = 0U; index < config.sample_count; ++index) {
    const double x =
        2.0 * static_cast<double>(index) / static_cast<double>(config.sample_count - 1U) - 1.0;
    const double phase =
        config.constant_rad +
        x * (config.linear_rad + x * (config.quadratic_rad + x * config.cubic_rad));
    diagnostics->normalized_aperture_coordinates.push_back(x);
    diagnostics->raw_phase_error_rad.push_back(phase);
    sum_x += x;
    sum_xx += x * x;
    sum_phase += phase;
    sum_x_phase += x * phase;
  }

  const double count = static_cast<double>(config.sample_count);
  const double determinant = count * sum_xx - sum_x * sum_x;
  diagnostics->fitted_unobservable_constant_rad =
      (sum_phase * sum_xx - sum_x_phase * sum_x) / determinant;
  diagnostics->fitted_unobservable_linear_rad =
      (count * sum_x_phase - sum_x * sum_phase) / determinant;

  diagnostics->unobservable_phase_rad.reserve(config.sample_count);
  diagnostics->observable_phase_error_rad.reserve(config.sample_count);
  diagnostics->correction_phase_rad.reserve(config.sample_count);
  double sum_observable = 0.0;
  double sum_x_observable = 0.0;
  double sum_observable_squared = 0.0;
  for (std::size_t index = 0U; index < config.sample_count; ++index) {
    const double x = diagnostics->normalized_aperture_coordinates[index];
    const double unobservable = diagnostics->fitted_unobservable_constant_rad +
                                diagnostics->fitted_unobservable_linear_rad * x;
    const double observable = diagnostics->raw_phase_error_rad[index] - unobservable;
    diagnostics->unobservable_phase_rad.push_back(unobservable);
    diagnostics->observable_phase_error_rad.push_back(observable);
    diagnostics->correction_phase_rad.push_back(-observable);
    sum_observable += observable;
    sum_x_observable += x * observable;
    sum_observable_squared += observable * observable;
    diagnostics->observable_max_abs_rad =
        std::max(diagnostics->observable_max_abs_rad, std::abs(observable));
  }

  diagnostics->observable_rms_rad = std::sqrt(sum_observable_squared / count);
  diagnostics->correction_rms_rad = diagnostics->observable_rms_rad;
  diagnostics->correction_max_abs_rad = diagnostics->observable_max_abs_rad;
  diagnostics->removal_residual_mean_rad = sum_observable / count;
  diagnostics->removal_residual_linear_projection_rad = sum_x_observable / sum_xx;
  diagnostics->valid = true;
  return true;
}

}  // namespace imaging
}  // namespace sar
