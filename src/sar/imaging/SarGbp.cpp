#include "sar/imaging/SarGbp.h"

#include <algorithm>
#include <cmath>

#include "sar/signal/SarWaveform.h"

namespace sar {
namespace imaging {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kSpeedOfLightMps = 299792458.0;
constexpr std::size_t kMaxApprovedDimension = 128U;

bool IsValid(const GbpConfig& config, const std::vector<geometry::PlatformPulseState>& pulses,
             const signal::ComplexMatrix& raw_pulse_history,
             const signal::ComplexVector& matched_filter) {
  return config.sample_rate_hz > 0.0 && config.carrier_frequency_hz > 0.0 &&
         config.grid.azimuth_pixel_count > 0U && config.grid.range_pixel_count > 0U &&
         config.grid.azimuth_pixel_count <= kMaxApprovedDimension &&
         config.grid.range_pixel_count <= kMaxApprovedDimension &&
         config.grid.azimuth_spacing_m > 0.0 && config.grid.range_spacing_m > 0.0 &&
         !pulses.empty() && pulses.size() == raw_pulse_history.rows &&
         raw_pulse_history.cols > 0U &&
         raw_pulse_history.values.size() == raw_pulse_history.rows * raw_pulse_history.cols &&
         !matched_filter.empty();
}

signal::ComplexSample InterpolateLinear(const signal::ComplexMatrix& matrix, std::size_t row,
                                        double source_col, bool* out_of_bounds) {
  if (source_col < 0.0 || source_col > static_cast<double>(matrix.cols - 1U)) {
    *out_of_bounds = true;
    return signal::ComplexSample(0.0, 0.0);
  }
  const std::size_t left = static_cast<std::size_t>(std::floor(source_col));
  const std::size_t right = std::min(left + 1U, matrix.cols - 1U);
  const double fraction = source_col - static_cast<double>(left);
  return matrix(row, left) * (1.0 - fraction) + matrix(row, right) * fraction;
}

bool RangeCompressHistory(const signal::ComplexMatrix& raw_pulse_history,
                          const signal::ComplexVector& matched_filter, double sample_rate_hz,
                          signal::ComplexMatrix* range_compressed) {
  range_compressed->rows = raw_pulse_history.rows;
  range_compressed->cols = raw_pulse_history.cols;
  range_compressed->values.assign(raw_pulse_history.values.size(), signal::ComplexSample(0.0, 0.0));
  for (std::size_t row = 0U; row < raw_pulse_history.rows; ++row) {
    signal::ComplexVector raw_row(raw_pulse_history.cols);
    for (std::size_t col = 0U; col < raw_pulse_history.cols; ++col) {
      raw_row[col] = raw_pulse_history(row, col);
    }
    signal::RangeCompressionResult compressed;
    if (!signal::RangeCompress(raw_row, matched_filter, sample_rate_hz, &compressed) ||
        compressed.range_aligned_output.size() != raw_pulse_history.cols) {
      return false;
    }
    for (std::size_t col = 0U; col < raw_pulse_history.cols; ++col) {
      (*range_compressed)(row, col) = compressed.range_aligned_output[col];
    }
  }
  return true;
}

}  // namespace

bool FocusSmallSceneGbp(const GbpConfig& config,
                        const std::vector<geometry::PlatformPulseState>& pulses,
                        const signal::ComplexMatrix& raw_pulse_history,
                        const signal::ComplexVector& matched_filter, FocusedGbpImage* output) {
  if (output == nullptr || !IsValid(config, pulses, raw_pulse_history, matched_filter)) {
    return false;
  }

  signal::ComplexMatrix range_compressed;
  if (!RangeCompressHistory(raw_pulse_history, matched_filter, config.sample_rate_hz,
                            &range_compressed)) {
    return false;
  }

  GbpDiagnostics diagnostics;
  diagnostics.evaluated_pixels = config.grid.azimuth_pixel_count * config.grid.range_pixel_count;
  output->image.rows = config.grid.azimuth_pixel_count;
  output->image.cols = config.grid.range_pixel_count;
  output->image.values.assign(diagnostics.evaluated_pixels, signal::ComplexSample(0.0, 0.0));

  const double wavelength_m = kSpeedOfLightMps / config.carrier_frequency_hz;
  for (std::size_t row = 0U; row < output->image.rows; ++row) {
    const double x_m =
        config.grid.azimuth_start_m + static_cast<double>(row) * config.grid.azimuth_spacing_m;
    for (std::size_t col = 0U; col < output->image.cols; ++col) {
      const double y_m =
          config.grid.range_start_m + static_cast<double>(col) * config.grid.range_spacing_m;
      geometry::LocalPoint pixel;
      pixel.x_m = x_m;
      pixel.y_m = y_m;
      pixel.z_m = config.grid.image_plane_z_m;
      signal::ComplexSample coherent_sum(0.0, 0.0);
      for (std::size_t pulse = 0U; pulse < pulses.size(); ++pulse) {
        const double slant_range_m = geometry::Distance(pulses[pulse].position_m, pixel);
        const double source_col = 2.0 * slant_range_m * config.sample_rate_hz / kSpeedOfLightMps;
        bool out_of_bounds = false;
        const signal::ComplexSample sample =
            InterpolateLinear(range_compressed, pulse, source_col, &out_of_bounds);
        if (out_of_bounds) {
          ++diagnostics.out_of_bounds_samples;
          continue;
        }
        const double phase = 4.0 * kPi * slant_range_m / wavelength_m;
        coherent_sum += sample * signal::ComplexSample(std::cos(phase), std::sin(phase));
        ++diagnostics.accumulated_samples;
      }
      output->image(row, col) = coherent_sum;
    }
  }
  output->diagnostics = diagnostics;
  return true;
}

}  // namespace imaging
}  // namespace sar
