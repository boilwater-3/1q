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

/**
 * @brief 修正贝塞尔函数 I0(x) 的级数展开(供 Kaiser 窗使用)。
 */
double BesselI0(double x) {
  double sum = 1.0;
  double term = 1.0;
  const double half_x = x * 0.5;
  for (std::size_t k = 1; k < 64U; ++k) {
    term *= (half_x / static_cast<double>(k)) * (half_x / static_cast<double>(k));
    sum += term;
    if (term < 1.0e-15 * sum) {
      break;
    }
  }
  return sum;
}

}  // namespace

bool GenerateLfmWaveform(const LfmWaveformConfig& config, LfmWaveform* waveform) {
  if (waveform == nullptr || !std::isfinite(config.bandwidth_hz) ||
      config.bandwidth_hz <= 0.0 || !std::isfinite(config.time_bandwidth_product) ||
      config.time_bandwidth_product <= 0.0 || !std::isfinite(config.sample_rate_hz) ||
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
  if (result == nullptr || input.empty() || matched_filter.empty() ||
      !std::isfinite(sample_rate_hz) || sample_rate_hz <= 0.0) {
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

bool RangeCompressRows(const ComplexMatrix& input, const ComplexVector& matched_filter,
                       double sample_rate_hz, ComplexMatrix* output) {
  if (output == nullptr || input.rows == 0U || input.cols == 0U ||
      input.values.size() != input.rows * input.cols || matched_filter.empty() ||
      !std::isfinite(sample_rate_hz) || sample_rate_hz <= 0.0) {
    return false;
  }

  const std::size_t convolution_size = input.cols + matched_filter.size() - 1U;
  const std::size_t fft_size = NextPowerOfTwo(convolution_size);

  // 匹配滤波器频谱与行无关:计算一次,各行复用(与逐行 RangeCompress 逐位一致)。
  ComplexVector padded_filter(fft_size, ComplexSample(0.0, 0.0));
  std::copy(matched_filter.begin(), matched_filter.end(), padded_filter.begin());
  ComplexVector filter_spectrum;
  if (!Fft1D(padded_filter, false, &filter_spectrum) || filter_spectrum.size() != fft_size) {
    return false;
  }

  output->rows = input.rows;
  output->cols = input.cols;
  output->values.assign(input.values.size(), ComplexSample(0.0, 0.0));

  ComplexVector padded_input(fft_size, ComplexSample(0.0, 0.0));
  ComplexVector input_spectrum;
  input_spectrum.reserve(fft_size);
  ComplexVector inverse;
  inverse.reserve(fft_size);
  const std::size_t aligned_start = matched_filter.size() - 1U;
  for (std::size_t row = 0; row < input.rows; ++row) {
    std::fill(padded_input.begin(), padded_input.end(), ComplexSample(0.0, 0.0));
    const ComplexSample* source = input.values.data() + row * input.cols;
    std::copy(source, source + input.cols, padded_input.begin());
    if (!Fft1D(padded_input, false, &input_spectrum) || input_spectrum.size() != fft_size) {
      return false;
    }
    for (std::size_t i = 0; i < fft_size; ++i) {
      input_spectrum[i] *= filter_spectrum[i];
    }
    if (!Fft1D(input_spectrum, true, &inverse) || inverse.size() != fft_size) {
      return false;
    }
    std::copy(inverse.begin() + static_cast<std::ptrdiff_t>(aligned_start),
              inverse.begin() + static_cast<std::ptrdiff_t>(aligned_start + input.cols),
              output->values.begin() + static_cast<std::ptrdiff_t>(row * input.cols));
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

bool GenerateWindow(const WindowSpec& spec, std::size_t length, ComplexVector* window) {
  if (window == nullptr || length == 0U) {
    return false;
  }
  window->assign(length, ComplexSample(0.0, 0.0));
  if (length == 1U) {
    (*window)[0] = ComplexSample(1.0, 0.0);
    return true;
  }
  const double denom = static_cast<double>(length - 1U);
  for (std::size_t n = 0; n < length; ++n) {
    double w = 1.0;
    const double phase_index = static_cast<double>(n);
    switch (spec.type) {
      case WindowType::kNone:
        w = 1.0;
        break;
      case WindowType::kHamming:
        w = 0.54 - 0.46 * std::cos(2.0 * kPi * phase_index / denom);
        break;
      case WindowType::kHanning:
        w = 0.5 - 0.5 * std::cos(2.0 * kPi * phase_index / denom);
        break;
      case WindowType::kBlackman:
        w = 0.42 - 0.5 * std::cos(2.0 * kPi * phase_index / denom) +
            0.08 * std::cos(4.0 * kPi * phase_index / denom);
        break;
      case WindowType::kKaiser: {
        const double beta = std::isfinite(spec.kaiser_beta) ? spec.kaiser_beta : 8.6;
        const double arg = (phase_index / denom - 0.5) * 2.0;  // [-1, 1]
        const double ratio = std::sqrt(std::max(1.0 - arg * arg, 0.0));
        w = BesselI0(beta * ratio) / BesselI0(beta);
        break;
      }
    }
    (*window)[n] = ComplexSample(w, 0.0);
  }
  return true;
}

bool BuildMatchedFilter(const ComplexVector& waveform, const WindowSpec& window,
                        ComplexVector* filter) {
  if (filter == nullptr || waveform.empty()) {
    return false;
  }
  ComplexVector window_samples;
  if (!GenerateWindow(window, waveform.size(), &window_samples)) {
    return false;
  }
  ComplexVector windowed_waveform(waveform.size());
  for (std::size_t i = 0; i < waveform.size(); ++i) {
    windowed_waveform[i] = waveform[i] * window_samples[i];
  }
  return BuildMatchedFilter(windowed_waveform, filter);
}

bool RangeCompress(const ComplexVector& input, const ComplexVector& matched_filter,
                   double sample_rate_hz, const WindowSpec& window,
                   RangeCompressionResult* result) {
  if (result == nullptr || input.empty() || matched_filter.empty()) {
    return false;
  }
  if (window.type == WindowType::kNone) {
    return RangeCompress(input, matched_filter, sample_rate_hz, result);
  }
  ComplexVector window_samples;
  if (!GenerateWindow(window, matched_filter.size(), &window_samples)) {
    return false;
  }
  ComplexVector windowed_filter(matched_filter.size());
  for (std::size_t i = 0; i < matched_filter.size(); ++i) {
    windowed_filter[i] = matched_filter[i] * window_samples[i];
  }
  return RangeCompress(input, windowed_filter, sample_rate_hz, result);
}

bool Compress2D(const ComplexMatrix& raw_pulse_history, const ComplexVector& range_matched_filter,
                const RangeAzimuthCompressionConfig& config, ComplexMatrix* output) {
  if (output == nullptr || raw_pulse_history.rows == 0U || raw_pulse_history.cols == 0U ||
      raw_pulse_history.values.size() != raw_pulse_history.rows * raw_pulse_history.cols ||
      range_matched_filter.empty() || !std::isfinite(config.sample_rate_hz) ||
      config.sample_rate_hz <= 0.0) {
    return false;
  }

  const std::size_t rows = raw_pulse_history.rows;
  const std::size_t cols = raw_pulse_history.cols;

  ComplexMatrix range_compressed;
  ComplexVector windowed_filter;
  const ComplexVector* effective_filter = &range_matched_filter;
  if (config.range_window.type != WindowType::kNone) {
    if (!GenerateWindow(config.range_window, range_matched_filter.size(), &windowed_filter)) {
      return false;
    }
    for (std::size_t i = 0; i < range_matched_filter.size(); ++i) {
      windowed_filter[i] = range_matched_filter[i] * windowed_filter[i];
    }
    effective_filter = &windowed_filter;
  }
  if (!RangeCompressRows(raw_pulse_history, *effective_filter, config.sample_rate_hz,
                         &range_compressed)) {
    return false;
  }

  const bool apply_azimuth =
      std::isfinite(config.azimuth_matched_filter_rate_hz_per_s) &&
      config.azimuth_matched_filter_rate_hz_per_s != 0.0 && rows > 1U &&
      std::isfinite(config.prf_hz) && config.prf_hz > 0.0;
  if (!apply_azimuth) {
    *output = range_compressed;
    return true;
  }

  // 方位压缩:逐列 FFT → 多普勒域匹配滤波 exp(j π fa² / Ka) → 逐列 IFFT。
  ComplexMatrix azimuth_spectrum;
  if (!FftCols(range_compressed, false, &azimuth_spectrum)) {
    return false;
  }

  const double ka = config.azimuth_matched_filter_rate_hz_per_s;
  ComplexVector azimuth_window_samples;
  if (config.azimuth_window.type != WindowType::kNone) {
    if (!GenerateWindow(config.azimuth_window, rows, &azimuth_window_samples)) {
      return false;
    }
  }

  const double row_count = static_cast<double>(rows);
  for (std::size_t col = 0; col < cols; ++col) {
    for (std::size_t row = 0; row < rows; ++row) {
      double fa_bin = static_cast<double>(row);
      if (fa_bin > row_count * 0.5) {
        fa_bin -= row_count;  // FFT bin → 双边多普勒频率
      }
      const double fa_hz = fa_bin * config.prf_hz / row_count;
      const double phase = kPi * fa_hz * fa_hz / ka;
      const ComplexSample matched_filter_value(std::cos(phase), std::sin(phase));
      ComplexSample sample = azimuth_spectrum(row, col) * matched_filter_value;
      if (config.azimuth_window.type != WindowType::kNone) {
        sample *= azimuth_window_samples[row];
      }
      azimuth_spectrum(row, col) = sample;
    }
  }

  return FftCols(azimuth_spectrum, true, output);
}

}  // namespace signal
}  // namespace sar
