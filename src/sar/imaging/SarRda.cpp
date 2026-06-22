#include "sar/imaging/SarRda.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "sar/geometry/SarGeometry.h"
#include "sar/imaging/SarImageQuality.h"
#include "sar/imaging/SarPhaseReference.h"
#include "sar/signal/SarWaveform.h"

namespace sar {
namespace imaging {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kSpeedOfLightMps = 299792458.0;

bool IsValid(const RdaConfig& config, const signal::ComplexMatrix& raw_pulse_history,
             const signal::ComplexVector& matched_filter) {
  return config.sample_rate_hz > 0.0 && config.carrier_frequency_hz > 0.0 && config.prf_hz > 0.0 &&
         config.platform_velocity_mps > 0.0 && config.reference_range_m > 0.0 &&
         raw_pulse_history.rows > 1U && raw_pulse_history.cols > 0U &&
         raw_pulse_history.values.size() == raw_pulse_history.rows * raw_pulse_history.cols &&
         !matched_filter.empty() &&
         (config.rcmc_interpolation != RcmcInterpolation::kSinc ||
          (config.sinc_half_width >= 2U && config.sinc_half_width <= 16U));
}

signal::ComplexSample InterpolateRangeLinear(const signal::ComplexMatrix& matrix, std::size_t row,
                                             double source_col, std::size_t* out_of_bounds) {
  if (source_col < 0.0 || source_col > static_cast<double>(matrix.cols - 1U)) {
    ++(*out_of_bounds);
    return signal::ComplexSample(0.0, 0.0);
  }

  const std::size_t left = static_cast<std::size_t>(std::floor(source_col));
  const std::size_t right = std::min(left + 1U, matrix.cols - 1U);
  const double frac = source_col - static_cast<double>(left);
  return matrix(row, left) * (1.0 - frac) + matrix(row, right) * frac;
}

signal::ComplexSample InterpolateRangeSinc(const signal::ComplexMatrix& matrix, std::size_t row,
                                           double source_col, std::size_t half_width,
                                           std::size_t* out_of_bounds) {
  if (source_col < 0.0 || source_col > static_cast<double>(matrix.cols - 1U)) {
    ++(*out_of_bounds);
    return signal::ComplexSample(0.0, 0.0);
  }

  const long center = static_cast<long>(std::floor(source_col));
  const long first = center - static_cast<long>(half_width) + 1L;
  const long last = center + static_cast<long>(half_width);
  signal::ComplexSample sum(0.0, 0.0);
  double weight_sum = 0.0;
  bool clipped = false;
  for (long index = first; index <= last; ++index) {
    if (index < 0L || index >= static_cast<long>(matrix.cols)) {
      clipped = true;
      continue;
    }
    const double distance = source_col - static_cast<double>(index);
    const double weight =
        geometry::Sinc(distance) * geometry::Sinc(distance / static_cast<double>(half_width));
    sum += matrix(row, static_cast<std::size_t>(index)) * weight;
    weight_sum += weight;
  }
  if (clipped) {
    ++(*out_of_bounds);
  }
  return std::abs(weight_sum) > 1.0e-12 ? sum / weight_sum : signal::ComplexSample(0.0, 0.0);
}

const char* InterpolationName(RcmcInterpolation interpolation) {
  switch (interpolation) {
    case RcmcInterpolation::kNone:
      return "none";
    case RcmcInterpolation::kLinear:
      return "linear";
    case RcmcInterpolation::kSinc:
      return "sinc";
  }
  return "unknown";
}

const char* PhaseReferenceModeName(PhaseReferenceMode mode) {
  switch (mode) {
    case PhaseReferenceMode::kNative:
      return "native";
    case PhaseReferenceMode::kCenterBroadside:
      return "center_broadside";
  }
  return "unknown";
}

}  // namespace

bool ComputeRdaSamplingDiagnostics(const RdaConfig& config, std::size_t pulse_count,
                                   RdaDiagnostics* diagnostics) {
  if (diagnostics == nullptr || pulse_count == 0U || config.sample_rate_hz <= 0.0 ||
      config.carrier_frequency_hz <= 0.0 || config.prf_hz <= 0.0 ||
      config.platform_velocity_mps <= 0.0 || config.reference_range_m <= 0.0) {
    return false;
  }

  *diagnostics = RdaDiagnostics{};
  diagnostics->reference_range_m = config.reference_range_m;
  diagnostics->range_bin_spacing_m = kSpeedOfLightMps / (2.0 * config.sample_rate_hz);
  const double wavelength_m = kSpeedOfLightMps / config.carrier_frequency_hz;
  diagnostics->doppler_rate_hz_per_s =
      2.0 * config.platform_velocity_mps * config.platform_velocity_mps /
      (wavelength_m * config.reference_range_m);
  diagnostics->azimuth_sample_spacing_m = config.platform_velocity_mps / config.prf_hz;
  diagnostics->azimuth_phase_curvature_rad_per_pulse2 =
      4.0 * kPi * diagnostics->azimuth_sample_spacing_m *
      diagnostics->azimuth_sample_spacing_m / (wavelength_m * config.reference_range_m);
  const double aperture_edge_m =
      0.5 * static_cast<double>(pulse_count - 1U) * diagnostics->azimuth_sample_spacing_m;
  const double aperture_half_pulses = 0.5 * static_cast<double>(pulse_count - 1U);
  diagnostics->azimuth_quadratic_phase_span_rad =
      diagnostics->azimuth_phase_curvature_rad_per_pulse2 *
      aperture_half_pulses * aperture_half_pulses;
  if (aperture_edge_m == 0.0) {
    diagnostics->max_geometric_doppler_hz = 0.0;
    diagnostics->doppler_nyquist_margin = std::numeric_limits<double>::infinity();
    return true;
  }

  diagnostics->max_geometric_doppler_hz =
      2.0 * config.platform_velocity_mps * aperture_edge_m /
      (wavelength_m *
       std::sqrt(config.reference_range_m * config.reference_range_m +
                 aperture_edge_m * aperture_edge_m));
  diagnostics->doppler_nyquist_margin =
      0.5 * config.prf_hz / diagnostics->max_geometric_doppler_hz;
  return true;
}

bool ApplyRangeMigrationCorrection(const signal::ComplexMatrix& input,
                                   const std::vector<double>& delta_bins_by_row,
                                   RcmcInterpolation interpolation, std::size_t sinc_half_width,
                                   signal::ComplexMatrix* output,
                                   std::size_t* out_of_bounds_samples) {
  if (output == nullptr || out_of_bounds_samples == nullptr || input.rows == 0U ||
      input.cols == 0U || input.values.size() != input.rows * input.cols ||
      delta_bins_by_row.size() != input.rows ||
      (interpolation == RcmcInterpolation::kSinc &&
       (sinc_half_width < 2U || sinc_half_width > 16U))) {
    return false;
  }

  *output = input;
  *out_of_bounds_samples = 0U;
  if (interpolation == RcmcInterpolation::kNone) {
    return true;
  }

  for (std::size_t row = 0U; row < input.rows; ++row) {
    for (std::size_t col = 0U; col < input.cols; ++col) {
      const double source_col = static_cast<double>(col) + delta_bins_by_row[row];
      (*output)(row, col) =
          interpolation == RcmcInterpolation::kLinear
              ? InterpolateRangeLinear(input, row, source_col, out_of_bounds_samples)
              : InterpolateRangeSinc(input, row, source_col, sinc_half_width,
                                     out_of_bounds_samples);
    }
  }
  return true;
}

bool FocusStripmapRda(const RdaConfig& config, const signal::ComplexMatrix& raw_pulse_history,
                      const signal::ComplexVector& matched_filter, FocusedSarImage* output) {
  if (output == nullptr || !IsValid(config, raw_pulse_history, matched_filter)) {
    return false;
  }

  RdaDiagnostics diagnostics;
  if (!ComputeRdaSamplingDiagnostics(config, raw_pulse_history.rows, &diagnostics)) {
    return false;
  }
  const double wavelength_m = kSpeedOfLightMps / config.carrier_frequency_hz;
  diagnostics.rcmc_interpolation = InterpolationName(config.rcmc_interpolation);

  signal::ComplexMatrix range_compressed;
  range_compressed.rows = raw_pulse_history.rows;
  range_compressed.cols = raw_pulse_history.cols;
  range_compressed.values.assign(raw_pulse_history.values.size(), signal::ComplexSample(0.0, 0.0));
  for (std::size_t row = 0U; row < raw_pulse_history.rows; ++row) {
    signal::ComplexVector raw_row(raw_pulse_history.cols);
    for (std::size_t col = 0U; col < raw_pulse_history.cols; ++col) {
      raw_row[col] = raw_pulse_history(row, col);
    }

    signal::RangeCompressionResult compression;
    if (!signal::RangeCompress(raw_row, matched_filter, config.sample_rate_hz, &compression) ||
        compression.range_aligned_output.size() != raw_pulse_history.cols) {
      return false;
    }
    for (std::size_t col = 0U; col < raw_pulse_history.cols; ++col) {
      range_compressed(row, col) = compression.range_aligned_output[col];
    }
  }
  diagnostics.range_compression_applied = true;

  PhaseReferenceConfig phase_reference_config;
  phase_reference_config.mode = PhaseReferenceMode::kCenterBroadside;
  phase_reference_config.carrier_frequency_hz = config.carrier_frequency_hz;
  phase_reference_config.prf_hz = config.prf_hz;
  phase_reference_config.platform_velocity_mps = config.platform_velocity_mps;
  phase_reference_config.range_bin_spacing_m = diagnostics.range_bin_spacing_m;
  PhaseReferenceDiagnostics phase_reference_diagnostics;
  if (!ApplyBroadsideCenterPhaseReference(phase_reference_config, &range_compressed,
                                          &phase_reference_diagnostics)) {
    return false;
  }
  diagnostics.phase_reference_applied = phase_reference_diagnostics.applied;
  diagnostics.phase_reference_mode = PhaseReferenceModeName(phase_reference_diagnostics.mode);

  signal::ComplexMatrix azimuth_spectrum;
  if (!signal::FftCols(range_compressed, false, &azimuth_spectrum)) {
    return false;
  }
  diagnostics.azimuth_fft_applied = true;

  std::vector<double> delta_bins_by_row(azimuth_spectrum.rows, 0.0);
  for (std::size_t row = 0U; row < azimuth_spectrum.rows; ++row) {
    const double fa_hz = geometry::DopplerBinFrequency(row, azimuth_spectrum.rows, config.prf_hz);
    const double delta_range_m =
        wavelength_m * wavelength_m * config.reference_range_m * fa_hz * fa_hz /
        (8.0 * config.platform_velocity_mps * config.platform_velocity_mps);
    delta_bins_by_row[row] = delta_range_m / diagnostics.range_bin_spacing_m;
  }
  signal::ComplexMatrix rcmc;
  if (!ApplyRangeMigrationCorrection(azimuth_spectrum, delta_bins_by_row, config.rcmc_interpolation,
                                     config.sinc_half_width, &rcmc,
                                     &diagnostics.out_of_bounds_samples)) {
    return false;
  }

  const double ka_hz_per_s = diagnostics.doppler_rate_hz_per_s;
  for (std::size_t row = 0U; row < rcmc.rows; ++row) {
    const double fa_hz = geometry::DopplerBinFrequency(row, rcmc.rows, config.prf_hz);
    const double phase = kPi * fa_hz * fa_hz / ka_hz_per_s;
    const signal::ComplexSample azimuth_filter(std::cos(phase), std::sin(phase));
    for (std::size_t col = 0U; col < rcmc.cols; ++col) {
      rcmc(row, col) *= azimuth_filter;
    }
  }
  diagnostics.azimuth_matched_filter_applied = true;

  signal::ComplexMatrix azimuth_time;
  if (!signal::FftCols(rcmc, true, &azimuth_time)) {
    return false;
  }
  diagnostics.azimuth_ifft_applied = true;

  output->image = azimuth_time;
  ImageQualityConfig quality_config;
  quality_config.range_pixel_spacing_m = diagnostics.range_bin_spacing_m;
  quality_config.azimuth_pixel_spacing_m = diagnostics.azimuth_sample_spacing_m;
  const ImageQualityMetrics quality = EvaluateImageQuality(output->image, quality_config);
  diagnostics.range_width_3db_bins = quality.range_width_3db_bins;
  diagnostics.azimuth_width_3db_bins = quality.azimuth_width_3db_bins;
  diagnostics.resolution_m_valid = quality.resolution_m_valid;
  diagnostics.range_resolution_3db_m = quality.range_resolution_3db_m;
  diagnostics.azimuth_resolution_3db_m = quality.azimuth_resolution_3db_m;
  diagnostics.image_entropy_nats = quality.entropy_nats;
  diagnostics.image_contrast = quality.image_contrast;
  output->diagnostics = diagnostics;
  return true;
}

std::size_t FindPeakIndex(const signal::ComplexMatrix& image) {
  std::size_t peak_index = 0U;
  double peak_magnitude = -1.0;
  for (std::size_t i = 0U; i < image.values.size(); ++i) {
    const double magnitude = std::abs(image.values[i]);
    if (magnitude > peak_magnitude) {
      peak_magnitude = magnitude;
      peak_index = i;
    }
  }
  return peak_index;
}

double EstimateAzimuthWidth3dbBins(const signal::ComplexMatrix& image) {
  return EvaluateImageQuality(image).azimuth_width_3db_bins;
}

double EstimateImageEntropyNats(const signal::ComplexMatrix& image) {
  return EvaluateImageQuality(image).entropy_nats;
}

}  // namespace imaging
}  // namespace sar
