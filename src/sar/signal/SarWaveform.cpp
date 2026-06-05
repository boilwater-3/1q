#include "sar/signal/SarWaveform.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sar {
namespace signal {

namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kTiny = 1.0e-30;

std::size_t NextPowerOfTwo(std::size_t value) {
  std::size_t result = 1U;
  while (result < value) {
    result <<= 1U;
  }
  return result;
}

std::size_t FindPeakIndex(const ComplexVector& values) {
  std::size_t peak_index = 0U;
  double peak_magnitude = -1.0;
  for (std::size_t i = 0; i < values.size(); ++i) {
    const double magnitude = std::abs(values[i]);
    if (magnitude > peak_magnitude) {
      peak_magnitude = magnitude;
      peak_index = i;
    }
  }
  return peak_index;
}

double ToDb(double linear_ratio) { return 10.0 * std::log10(std::max(linear_ratio, kTiny)); }

}  // namespace

bool GenerateLfmWaveform(const LfmWaveformConfig& config, LfmWaveform* waveform) {
  if (waveform == nullptr || config.bandwidth_hz <= 0.0 || config.time_bandwidth_product <= 0.0 ||
      config.sample_rate_hz <= 0.0) {
    return false;
  }

  const double pulse_width_s = config.time_bandwidth_product / config.bandwidth_hz;
  const double chirp_rate_hz_per_s = config.bandwidth_hz / pulse_width_s;
  const std::size_t sample_count =
      static_cast<std::size_t>(std::ceil(pulse_width_s * config.sample_rate_hz));
  if (sample_count == 0U) {
    return false;
  }

  waveform->config = config;
  waveform->pulse_width_s = pulse_width_s;
  waveform->chirp_rate_hz_per_s = chirp_rate_hz_per_s;
  waveform->samples.assign(sample_count, ComplexSample(0.0, 0.0));

  for (std::size_t n = 0; n < sample_count; ++n) {
    const double t = static_cast<double>(n) / config.sample_rate_hz;
    const double phase =
        2.0 * kPi * (config.start_frequency_hz * t + 0.5 * chirp_rate_hz_per_s * t * t);
    waveform->samples[n] = ComplexSample(std::cos(phase), std::sin(phase));
  }
  return true;
}

bool BuildMatchedFilter(const ComplexVector& waveform, ComplexVector* filter) {
  if (filter == nullptr || waveform.empty()) {
    return false;
  }
  filter->assign(waveform.size(), ComplexSample(0.0, 0.0));
  for (std::size_t i = 0; i < waveform.size(); ++i) {
    (*filter)[i] = std::conj(waveform[waveform.size() - 1U - i]);
  }
  return true;
}

bool LinearConvolveFft(const ComplexVector& input, const ComplexVector& filter,
                       ComplexVector* output) {
  if (output == nullptr || input.empty() || filter.empty()) {
    return false;
  }

  const std::size_t convolution_size = input.size() + filter.size() - 1U;
  const std::size_t fft_size = NextPowerOfTwo(convolution_size);
  ComplexVector padded_input(fft_size, ComplexSample(0.0, 0.0));
  ComplexVector padded_filter(fft_size, ComplexSample(0.0, 0.0));
  std::copy(input.begin(), input.end(), padded_input.begin());
  std::copy(filter.begin(), filter.end(), padded_filter.begin());

  ComplexVector input_spectrum;
  ComplexVector filter_spectrum;
  if (!Fft1D(padded_input, false, &input_spectrum) ||
      !Fft1D(padded_filter, false, &filter_spectrum) || input_spectrum.size() != fft_size ||
      filter_spectrum.size() != fft_size) {
    return false;
  }

  ComplexVector product(fft_size, ComplexSample(0.0, 0.0));
  for (std::size_t i = 0; i < fft_size; ++i) {
    product[i] = input_spectrum[i] * filter_spectrum[i];
  }

  ComplexVector inverse;
  if (!Fft1D(product, true, &inverse) || inverse.size() != fft_size) {
    return false;
  }

  output->assign(inverse.begin(), inverse.begin() + static_cast<std::ptrdiff_t>(convolution_size));
  return true;
}

bool RangeCompress(const ComplexVector& input, const ComplexVector& matched_filter,
                   double sample_rate_hz, RangeCompressionResult* result) {
  if (result == nullptr || input.empty() || matched_filter.empty() || sample_rate_hz <= 0.0) {
    return false;
  }

  ComplexVector full_convolution;
  if (!LinearConvolveFft(input, matched_filter, &full_convolution)) {
    return false;
  }

  result->full_convolution = full_convolution;
  result->range_bin_spacing_m = kSpeedOfLightMps / (2.0 * sample_rate_hz);
  result->full_peak_index = FindPeakIndex(full_convolution);
  result->aligned_peak_index = (result->full_peak_index >= matched_filter.size() - 1U)
                                   ? (result->full_peak_index - (matched_filter.size() - 1U))
                                   : 0U;

  result->range_aligned_output.assign(input.size(), ComplexSample(0.0, 0.0));
  const std::size_t start = matched_filter.size() - 1U;
  for (std::size_t i = 0; i < input.size(); ++i) {
    const std::size_t source_index = start + i;
    if (source_index < full_convolution.size()) {
      result->range_aligned_output[i] = full_convolution[source_index];
    }
  }
  return true;
}

bool EstimatePulseQuality(const ComplexVector& compressed_pulse, PulseQualityMetrics* metrics) {
  if (metrics == nullptr || compressed_pulse.empty()) {
    return false;
  }

  const std::size_t peak_index = FindPeakIndex(compressed_pulse);
  const double peak_magnitude = std::abs(compressed_pulse[peak_index]);
  if (peak_magnitude <= 0.0 || !std::isfinite(peak_magnitude)) {
    return false;
  }

  const double half_power_magnitude = peak_magnitude / std::sqrt(2.0);
  std::size_t main_lobe_start = peak_index;
  while (main_lobe_start > 0U &&
         std::abs(compressed_pulse[main_lobe_start - 1U]) >= half_power_magnitude) {
    --main_lobe_start;
  }
  std::size_t main_lobe_end = peak_index;
  while (main_lobe_end + 1U < compressed_pulse.size() &&
         std::abs(compressed_pulse[main_lobe_end + 1U]) >= half_power_magnitude) {
    ++main_lobe_end;
  }

  const double twenty_db_magnitude = peak_magnitude * 0.1;
  std::size_t twenty_db_start = peak_index;
  while (twenty_db_start > 0U &&
         std::abs(compressed_pulse[twenty_db_start - 1U]) >= twenty_db_magnitude) {
    --twenty_db_start;
  }
  std::size_t twenty_db_end = peak_index;
  while (twenty_db_end + 1U < compressed_pulse.size() &&
         std::abs(compressed_pulse[twenty_db_end + 1U]) >= twenty_db_magnitude) {
    ++twenty_db_end;
  }

  double main_lobe_energy = 0.0;
  double side_lobe_energy = 0.0;
  double max_side_lobe_power = 0.0;
  for (std::size_t i = 0; i < compressed_pulse.size(); ++i) {
    const double power = std::norm(compressed_pulse[i]);
    if (i >= main_lobe_start && i <= main_lobe_end) {
      main_lobe_energy += power;
    } else {
      side_lobe_energy += power;
      max_side_lobe_power = std::max(max_side_lobe_power, power);
    }
  }

  metrics->peak_index = peak_index;
  metrics->peak_magnitude = peak_magnitude;
  metrics->main_lobe_start = main_lobe_start;
  metrics->main_lobe_end = main_lobe_end;
  metrics->width_3db_bins = static_cast<double>(main_lobe_end - main_lobe_start + 1U);
  metrics->width_20db_bins = static_cast<double>(twenty_db_end - twenty_db_start + 1U);
  metrics->pslr_db = ToDb(max_side_lobe_power / std::max(peak_magnitude * peak_magnitude, kTiny));
  metrics->islr_db = ToDb(side_lobe_energy / std::max(main_lobe_energy, kTiny));
  return true;
}

}  // namespace signal
}  // namespace sar
