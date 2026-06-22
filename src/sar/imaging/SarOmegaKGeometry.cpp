// ============================================================================
// 【未进行设计需求，不再扩展 — DEPRECATED】
// 本文件不参与构建（见 src/sar/CMakeLists.txt 的 SAR_ENGINE_SOURCES 注释），
// 仅作为探索性参考保留。请勿新增依赖或据此实施。
// ============================================================================

#include "sar/imaging/SarOmegaKGeometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sar {
namespace imaging {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kSpeedOfLightMps = 299792458.0;

bool IsPositiveFinite(double value) {
  return std::isfinite(value) && value > 0.0;
}

double UnshiftedFrequency(std::size_t index, std::size_t count, double rate_hz) {
  const double raw = static_cast<double>(index) * rate_hz / static_cast<double>(count);
  return index <= count / 2U ? raw : raw - rate_hz;
}

}  // namespace

bool EvaluateOmegaKStoltGeometry(const OmegaKGeometryConfig& config,
                                 OmegaKGeometryDiagnostics* diagnostics) {
  if (diagnostics == nullptr) {
    return false;
  }
  *diagnostics = OmegaKGeometryDiagnostics{};
  if (config.range_sample_count == 0U || config.azimuth_pulse_count == 0U ||
      !IsPositiveFinite(config.sample_rate_hz) || !IsPositiveFinite(config.prf_hz) ||
      !IsPositiveFinite(config.carrier_frequency_hz) ||
      !IsPositiveFinite(config.platform_velocity_mps) ||
      !IsPositiveFinite(config.reference_range_m)) {
    return false;
  }

  diagnostics->range_frequency_bin_count = config.range_sample_count;
  diagnostics->azimuth_frequency_bin_count = config.azimuth_pulse_count;
  diagnostics->range_frequency_spacing_hz =
      config.sample_rate_hz / static_cast<double>(config.range_sample_count);
  diagnostics->azimuth_frequency_spacing_hz =
      config.prf_hz / static_cast<double>(config.azimuth_pulse_count);
  diagnostics->wavelength_m = kSpeedOfLightMps / config.carrier_frequency_hz;
  diagnostics->minimum_range_wavenumber_rad_per_m = std::numeric_limits<double>::infinity();
  diagnostics->minimum_valid_propagation_wavenumber_rad_per_m =
      std::numeric_limits<double>::infinity();
  diagnostics->minimum_source_range_frequency_hz = std::numeric_limits<double>::infinity();
  diagnostics->maximum_source_range_frequency_hz = -std::numeric_limits<double>::infinity();

  diagnostics->range_frequencies_hz.reserve(config.range_sample_count);
  diagnostics->range_wavenumbers_rad_per_m.reserve(config.range_sample_count);
  double minimum_supported_range_frequency_hz = std::numeric_limits<double>::infinity();
  double maximum_supported_range_frequency_hz = -std::numeric_limits<double>::infinity();
  for (std::size_t range_index = 0U; range_index < config.range_sample_count; ++range_index) {
    const double frequency_hz =
        UnshiftedFrequency(range_index, config.range_sample_count, config.sample_rate_hz);
    const double wavenumber =
        4.0 * kPi * (config.carrier_frequency_hz + frequency_hz) / kSpeedOfLightMps;
    diagnostics->range_frequencies_hz.push_back(frequency_hz);
    diagnostics->range_wavenumbers_rad_per_m.push_back(wavenumber);
    minimum_supported_range_frequency_hz =
        std::min(minimum_supported_range_frequency_hz, frequency_hz);
    maximum_supported_range_frequency_hz =
        std::max(maximum_supported_range_frequency_hz, frequency_hz);
    diagnostics->minimum_range_wavenumber_rad_per_m =
        std::min(diagnostics->minimum_range_wavenumber_rad_per_m, wavenumber);
    diagnostics->maximum_range_wavenumber_rad_per_m =
        std::max(diagnostics->maximum_range_wavenumber_rad_per_m, wavenumber);
  }

  diagnostics->azimuth_frequencies_hz.reserve(config.azimuth_pulse_count);
  diagnostics->azimuth_wavenumbers_rad_per_m.reserve(config.azimuth_pulse_count);
  for (std::size_t azimuth_index = 0U; azimuth_index < config.azimuth_pulse_count;
       ++azimuth_index) {
    const double frequency_hz =
        UnshiftedFrequency(azimuth_index, config.azimuth_pulse_count, config.prf_hz);
    const double wavenumber = 2.0 * kPi * frequency_hz / config.platform_velocity_mps;
    diagnostics->azimuth_frequencies_hz.push_back(frequency_hz);
    diagnostics->azimuth_wavenumbers_rad_per_m.push_back(wavenumber);
    diagnostics->maximum_abs_azimuth_wavenumber_rad_per_m =
        std::max(diagnostics->maximum_abs_azimuth_wavenumber_rad_per_m,
                 std::abs(wavenumber));
  }

  const std::size_t point_count = config.range_sample_count * config.azimuth_pulse_count;
  diagnostics->propagation_wavenumbers_rad_per_m.reserve(point_count);
  diagnostics->source_range_frequency_queries_hz.reserve(point_count);
  diagnostics->stolt_shifts_hz.reserve(point_count);
  for (std::size_t azimuth_index = 0U; azimuth_index < config.azimuth_pulse_count;
       ++azimuth_index) {
    const double azimuth_wavenumber =
        diagnostics->azimuth_wavenumbers_rad_per_m[azimuth_index];
    for (std::size_t range_index = 0U; range_index < config.range_sample_count; ++range_index) {
      const double range_wavenumber = diagnostics->range_wavenumbers_rad_per_m[range_index];
      const double dispersion = range_wavenumber * range_wavenumber -
                                azimuth_wavenumber * azimuth_wavenumber;
      if (range_wavenumber <= 0.0 || dispersion <= 0.0) {
        ++diagnostics->invalid_dispersion_point_count;
        diagnostics->propagation_wavenumbers_rad_per_m.push_back(0.0);
      } else {
        const double propagation_wavenumber = std::sqrt(dispersion);
        diagnostics->propagation_wavenumbers_rad_per_m.push_back(propagation_wavenumber);
        diagnostics->minimum_valid_propagation_wavenumber_rad_per_m =
            std::min(diagnostics->minimum_valid_propagation_wavenumber_rad_per_m,
                     propagation_wavenumber);
        diagnostics->maximum_valid_propagation_wavenumber_rad_per_m =
            std::max(diagnostics->maximum_valid_propagation_wavenumber_rad_per_m,
                     propagation_wavenumber);
      }

      const double target_propagation_wavenumber = range_wavenumber;
      const double source_range_wavenumber =
          std::sqrt(target_propagation_wavenumber * target_propagation_wavenumber +
                    azimuth_wavenumber * azimuth_wavenumber);
      const double source_frequency_hz =
          azimuth_wavenumber == 0.0
              ? diagnostics->range_frequencies_hz[range_index]
              : kSpeedOfLightMps * source_range_wavenumber / (4.0 * kPi) -
                    config.carrier_frequency_hz;
      const double shift_hz = source_frequency_hz - diagnostics->range_frequencies_hz[range_index];
      diagnostics->source_range_frequency_queries_hz.push_back(source_frequency_hz);
      diagnostics->stolt_shifts_hz.push_back(shift_hz);
      diagnostics->minimum_source_range_frequency_hz =
          std::min(diagnostics->minimum_source_range_frequency_hz, source_frequency_hz);
      diagnostics->maximum_source_range_frequency_hz =
          std::max(diagnostics->maximum_source_range_frequency_hz, source_frequency_hz);
      diagnostics->maximum_abs_stolt_shift_hz =
          std::max(diagnostics->maximum_abs_stolt_shift_hz, std::abs(shift_hz));
      if (source_frequency_hz < minimum_supported_range_frequency_hz ||
          source_frequency_hz > maximum_supported_range_frequency_hz) {
        ++diagnostics->out_of_support_stolt_query_count;
      }
    }
  }

  diagnostics->valid = diagnostics->invalid_dispersion_point_count == 0U &&
                       diagnostics->out_of_support_stolt_query_count == 0U;
  return true;
}

}  // namespace imaging
}  // namespace sar
