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

enum class BackprojectionTraversal { kPixelMajor, kPulseMajor };

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
  // 批量距离压缩:匹配滤波器频谱只算一次(与逐行 RangeCompress 逐位一致)。
  return signal::RangeCompressRows(raw_pulse_history, matched_filter, sample_rate_hz,
                                   range_compressed);
}

geometry::LocalPoint MakePixel(const GbpConfig& config, std::size_t row, std::size_t col) {
  geometry::LocalPoint pixel;
  pixel.x_m =
      config.grid.azimuth_start_m + static_cast<double>(row) * config.grid.azimuth_spacing_m;
  pixel.y_m = config.grid.range_start_m + static_cast<double>(col) * config.grid.range_spacing_m;
  pixel.z_m = config.grid.image_plane_z_m;
  return pixel;
}

void AccumulatePulseAtPixel(const GbpConfig& config,
                            const std::vector<geometry::PlatformPulseState>& pulses,
                            const signal::ComplexMatrix& range_compressed, double wavelength_m,
                            std::size_t pulse, std::size_t row, std::size_t col,
                            FocusedGbpImage* output, GbpDiagnostics* diagnostics) {
  const geometry::LocalPoint pixel = MakePixel(config, row, col);
  const double slant_range_m = geometry::Distance(pulses[pulse].position_m, pixel);
  const double source_col = 2.0 * slant_range_m * config.sample_rate_hz / kSpeedOfLightMps;
  bool out_of_bounds = false;
  const signal::ComplexSample sample =
      InterpolateLinear(range_compressed, pulse, source_col, &out_of_bounds);
  if (out_of_bounds) {
    ++diagnostics->out_of_bounds_samples;
    return;
  }
  const double phase = 4.0 * kPi * slant_range_m / wavelength_m;
  output->image(row, col) += sample * signal::ComplexSample(std::cos(phase), std::sin(phase));
  ++diagnostics->accumulated_samples;
}

bool FocusSmallSceneBackprojection(const GbpConfig& config,
                                   const std::vector<geometry::PlatformPulseState>& pulses,
                                   const signal::ComplexMatrix& raw_pulse_history,
                                   const signal::ComplexVector& matched_filter,
                                   BackprojectionTraversal traversal, FocusedGbpImage* output) {
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
  diagnostics.traversal_order =
      traversal == BackprojectionTraversal::kPixelMajor ? "pixel_major" : "pulse_major";
  output->image.rows = config.grid.azimuth_pixel_count;
  output->image.cols = config.grid.range_pixel_count;
  output->image.values.assign(diagnostics.evaluated_pixels, signal::ComplexSample(0.0, 0.0));

  const double wavelength_m = kSpeedOfLightMps / config.carrier_frequency_hz;
  if (traversal == BackprojectionTraversal::kPixelMajor) {
    for (std::size_t row = 0U; row < output->image.rows; ++row) {
      for (std::size_t col = 0U; col < output->image.cols; ++col) {
        for (std::size_t pulse = 0U; pulse < pulses.size(); ++pulse) {
          AccumulatePulseAtPixel(config, pulses, range_compressed, wavelength_m, pulse, row, col,
                                 output, &diagnostics);
        }
      }
    }
  } else {
    for (std::size_t pulse = 0U; pulse < pulses.size(); ++pulse) {
      for (std::size_t row = 0U; row < output->image.rows; ++row) {
        for (std::size_t col = 0U; col < output->image.cols; ++col) {
          AccumulatePulseAtPixel(config, pulses, range_compressed, wavelength_m, pulse, row, col,
                                 output, &diagnostics);
        }
      }
    }
  }
  output->diagnostics = diagnostics;
  return true;
}

}  // namespace

bool FocusSmallSceneGbp(const GbpConfig& config,
                        const std::vector<geometry::PlatformPulseState>& pulses,
                        const signal::ComplexMatrix& raw_pulse_history,
                        const signal::ComplexVector& matched_filter, FocusedGbpImage* output) {
  return FocusSmallSceneBackprojection(config, pulses, raw_pulse_history, matched_filter,
                                       BackprojectionTraversal::kPixelMajor, output);
}

bool FocusSmallSceneBp(const GbpConfig& config,
                       const std::vector<geometry::PlatformPulseState>& pulses,
                       const signal::ComplexMatrix& raw_pulse_history,
                       const signal::ComplexVector& matched_filter, FocusedGbpImage* output) {
  return FocusSmallSceneBackprojection(config, pulses, raw_pulse_history, matched_filter,
                                       BackprojectionTraversal::kPulseMajor, output);
}

}  // namespace imaging
}  // namespace sar
