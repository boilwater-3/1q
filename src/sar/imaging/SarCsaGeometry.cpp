// ============================================================================
// 【未进行设计需求，不再扩展 — DEPRECATED】
// 本文件不参与构建（见 src/sar/CMakeLists.txt 的 SAR_ENGINE_SOURCES 注释），
// 仅作为探索性参考保留。请勿新增依赖或据此实施。
// ============================================================================

#include "sar/imaging/SarCsaGeometry.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include "common/numerics/Constants.h"

namespace sar {
namespace imaging {

namespace {

using oneq::common::numerics::kLightSpeed;

bool IsPositiveFinite(double value) {
  return std::isfinite(value) && value > 0.0;
}

double UnshiftedFrequency(std::size_t index, std::size_t count, double rate_hz) {
  const double raw = static_cast<double>(index) * rate_hz / static_cast<double>(count);
  return index <= count / 2U ? raw : raw - rate_hz;
}

}  // namespace

bool EvaluateCsaFrequencyGeometry(const CsaGeometryConfig& config,
                                  CsaGeometryDiagnostics* diagnostics) {
  if (diagnostics == nullptr) {
    return false;
  }
  *diagnostics = CsaGeometryDiagnostics{};
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
  diagnostics->wavelength_m = kLightSpeed / config.carrier_frequency_hz;
  diagnostics->valid_doppler_limit_hz =
      2.0 * config.platform_velocity_mps / diagnostics->wavelength_m;
  diagnostics->minimum_doppler_factor = std::numeric_limits<double>::infinity();

  diagnostics->range_frequencies_hz.reserve(config.range_sample_count);
  for (std::size_t index = 0U; index < config.range_sample_count; ++index) {
    diagnostics->range_frequencies_hz.push_back(
        UnshiftedFrequency(index, config.range_sample_count, config.sample_rate_hz));
  }

  diagnostics->azimuth_frequencies_hz.reserve(config.azimuth_pulse_count);
  diagnostics->doppler_factors.reserve(config.azimuth_pulse_count);
  diagnostics->chirp_scaling_factors.reserve(config.azimuth_pulse_count);
  for (std::size_t index = 0U; index < config.azimuth_pulse_count; ++index) {
    const double frequency_hz =
        UnshiftedFrequency(index, config.azimuth_pulse_count, config.prf_hz);
    diagnostics->azimuth_frequencies_hz.push_back(frequency_hz);
    diagnostics->maximum_abs_azimuth_frequency_hz =
        std::max(diagnostics->maximum_abs_azimuth_frequency_hz, std::abs(frequency_hz));

    const double normalized_doppler = frequency_hz / diagnostics->valid_doppler_limit_hz;
    if (std::abs(normalized_doppler) >= 1.0) {
      ++diagnostics->invalid_doppler_bin_count;
      diagnostics->doppler_factors.push_back(0.0);
      diagnostics->chirp_scaling_factors.push_back(0.0);
      continue;
    }

    const double doppler_factor = std::sqrt(1.0 - normalized_doppler * normalized_doppler);
    const double chirp_scaling_factor = 1.0 / doppler_factor - 1.0;
    diagnostics->doppler_factors.push_back(doppler_factor);
    diagnostics->chirp_scaling_factors.push_back(chirp_scaling_factor);
    diagnostics->minimum_doppler_factor =
        std::min(diagnostics->minimum_doppler_factor, doppler_factor);
    diagnostics->maximum_abs_chirp_scaling_factor =
        std::max(diagnostics->maximum_abs_chirp_scaling_factor,
                 std::abs(chirp_scaling_factor));
  }

  diagnostics->doppler_validity_margin_hz =
      diagnostics->valid_doppler_limit_hz - diagnostics->maximum_abs_azimuth_frequency_hz;
  diagnostics->valid = diagnostics->invalid_doppler_bin_count == 0U;
  return true;
}

}  // namespace imaging
}  // namespace sar
