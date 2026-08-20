#include "sar/imaging/SarMotionCompensation.h"

#include <algorithm>
#include <cmath>

namespace sar {
namespace imaging {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kSpeedOfLightMps = 299792458.0;

signal::ComplexSample InterpolateLinear(const signal::ComplexMatrix& matrix, std::size_t row,
                                        double source_col, std::size_t* out_of_bounds_samples) {
  if (source_col < 0.0 || source_col > static_cast<double>(matrix.cols - 1U)) {
    ++(*out_of_bounds_samples);
    return signal::ComplexSample(0.0, 0.0);
  }
  const std::size_t left = static_cast<std::size_t>(std::floor(source_col));
  const std::size_t right = std::min(left + 1U, matrix.cols - 1U);
  const double fraction = source_col - static_cast<double>(left);
  return matrix(row, left) * (1.0 - fraction) + matrix(row, right) * fraction;
}

}  // namespace

bool ApplyFirstOrderMotionCompensation(
    const FirstOrderMotionCompensationConfig& config,
    const std::vector<geometry::PlatformPulseState>& ideal_trajectory,
    const std::vector<geometry::PlatformPulseState>& actual_trajectory,
    const signal::ComplexMatrix& actual_raw_pulse_history, signal::ComplexMatrix* compensated,
    MotionCompensationDiagnostics* diagnostics) {
  if (compensated == nullptr || diagnostics == nullptr || config.sample_rate_hz <= 0.0 ||
      config.carrier_frequency_hz <= 0.0 || ideal_trajectory.empty() ||
      ideal_trajectory.size() != actual_trajectory.size() ||
      ideal_trajectory.size() != actual_raw_pulse_history.rows ||
      actual_raw_pulse_history.cols == 0U ||
      actual_raw_pulse_history.values.size() !=
          actual_raw_pulse_history.rows * actual_raw_pulse_history.cols) {
    return false;
  }

  compensated->rows = actual_raw_pulse_history.rows;
  compensated->cols = actual_raw_pulse_history.cols;
  compensated->values.assign(actual_raw_pulse_history.values.size(),
                             signal::ComplexSample(0.0, 0.0));
  *diagnostics = MotionCompensationDiagnostics{};
  double range_error_squared_sum = 0.0;
  const double wavelength_m = kSpeedOfLightMps / config.carrier_frequency_hz;

  for (std::size_t row = 0U; row < actual_raw_pulse_history.rows; ++row) {
    const double ideal_range_m =
        geometry::Distance(ideal_trajectory[row].position_m, config.reference_point_m);
    const double actual_range_m =
        geometry::Distance(actual_trajectory[row].position_m, config.reference_point_m);
    const double range_error_m = actual_range_m - ideal_range_m;
    const double envelope_shift_bins =
        2.0 * range_error_m * config.sample_rate_hz / kSpeedOfLightMps;
    const double phase = 4.0 * kPi * range_error_m / wavelength_m;
    const signal::ComplexSample phase_correction(std::cos(phase), std::sin(phase));

    diagnostics->max_abs_range_error_m =
        std::max(diagnostics->max_abs_range_error_m, std::abs(range_error_m));
    diagnostics->max_abs_envelope_shift_bins =
        std::max(diagnostics->max_abs_envelope_shift_bins, std::abs(envelope_shift_bins));
    range_error_squared_sum += range_error_m * range_error_m;
    for (std::size_t col = 0U; col < actual_raw_pulse_history.cols; ++col) {
      const double source_col = static_cast<double>(col) + envelope_shift_bins;
      (*compensated)(row, col) = InterpolateLinear(actual_raw_pulse_history, row, source_col,
                                                   &diagnostics->out_of_bounds_samples) *
                                 phase_correction;
    }
    ++diagnostics->compensated_pulses;
  }
  diagnostics->rms_range_error_m =
      std::sqrt(range_error_squared_sum / static_cast<double>(actual_raw_pulse_history.rows));
  return true;
}

}  // namespace imaging
}  // namespace sar
