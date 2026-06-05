/**
 * @file SarWaveform.h
 * @brief SAR 内部 LFM 波形、匹配滤波和距离压缩工具。
 */

#ifndef ONEQ_SRC_SAR_SIGNAL_SAR_WAVEFORM_H_
#define ONEQ_SRC_SAR_SIGNAL_SAR_WAVEFORM_H_

#include <cstddef>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace signal {

/**
 * @brief LFM 波形配置。
 */
struct LfmWaveformConfig {
  double bandwidth_hz{0.0};
  double time_bandwidth_product{0.0};
  double sample_rate_hz{0.0};
  double start_frequency_hz{0.0};
};

/**
 * @brief LFM 波形及派生参数。
 */
struct LfmWaveform {
  LfmWaveformConfig config{};
  double pulse_width_s{0.0};
  double chirp_rate_hz_per_s{0.0};
  ComplexVector samples{};
};

/**
 * @brief 距离压缩输出。
 */
struct RangeCompressionResult {
  ComplexVector full_convolution{};
  ComplexVector range_aligned_output{};
  double range_bin_spacing_m{0.0};
  std::size_t full_peak_index{0U};
  std::size_t aligned_peak_index{0U};
};

/**
 * @brief 压缩脉冲质量指标。
 */
struct PulseQualityMetrics {
  std::size_t peak_index{0U};
  double peak_magnitude{0.0};
  std::size_t main_lobe_start{0U};
  std::size_t main_lobe_end{0U};
  double width_3db_bins{0.0};
  double width_20db_bins{0.0};
  double pslr_db{0.0};
  double islr_db{0.0};
};

/**
 * @brief 生成 LFM 波形。
 */
bool GenerateLfmWaveform(const LfmWaveformConfig& config, LfmWaveform* waveform);

/**
 * @brief 构造时域匹配滤波器 `h[n] = conj(s[N - 1 - n])`。
 */
bool BuildMatchedFilter(const ComplexVector& waveform, ComplexVector* filter);

/**
 * @brief 计算 FFT 线性卷积。
 */
bool LinearConvolveFft(const ComplexVector& input, const ComplexVector& filter,
                       ComplexVector* output);

/**
 * @brief 执行距离向脉冲压缩。
 */
bool RangeCompress(const ComplexVector& input, const ComplexVector& matched_filter,
                   double sample_rate_hz, RangeCompressionResult* result);

/**
 * @brief 估计压缩脉冲质量指标。
 */
bool EstimatePulseQuality(const ComplexVector& compressed_pulse, PulseQualityMetrics* metrics);

}  // namespace signal
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SIGNAL_SAR_WAVEFORM_H_
