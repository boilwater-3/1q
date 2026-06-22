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

/**
 * @brief 窗函数类型(用于距离/方位脉冲压缩的旁瓣抑制)。
 */
enum class WindowType {
  kNone = 0,
  kHamming = 1,
  kHanning = 2,
  kBlackman = 3,
  kKaiser = 4,
};

/**
 * @brief 窗函数规格。
 */
struct WindowSpec {
  WindowType type{WindowType::kNone};
  double kaiser_beta{8.6};  ///< 仅 kKaiser 使用
};

/**
 * @brief 生成长度为 length 的复数窗(实部为窗值,虚部为 0)。
 */
bool GenerateWindow(const WindowSpec& spec, std::size_t length, ComplexVector* window);

/**
 * @brief 对发射波形施加窗后构造匹配滤波器(加窗降低旁瓣)。
 *        等价于先 windowed[n] = waveform[n] * window[n],再 h[n] = conj(windowed[N-1-n])。
 */
bool BuildMatchedFilter(const ComplexVector& waveform, const WindowSpec& window,
                        ComplexVector* filter);

/**
 * @brief 加窗距离向脉冲压缩:对 matched_filter 施加窗后压缩。
 *        原 RangeCompress(input, filter, fs, result) 等价于 kNone 窗路径。
 */
bool RangeCompress(const ComplexVector& input, const ComplexVector& matched_filter,
                   double sample_rate_hz, const WindowSpec& window, RangeCompressionResult* result);

/**
 * @brief 二维(距离 + 方位)脉冲压缩配置。
 */
struct RangeAzimuthCompressionConfig {
  double sample_rate_hz{0.0};
  double prf_hz{0.0};
  WindowSpec range_window{};
  WindowSpec azimuth_window{};
  double azimuth_matched_filter_rate_hz_per_s{0.0};  ///< 多普勒调频率 Ka,非零则执行方位压缩
};

/**
 * @brief 二维脉冲压缩:逐行距离压缩,可选方位匹配滤波(多普勒域)。
 *        raw_pulse_history 行 = 脉冲(方位),列 = 距离采样。
 *        方位压缩按距离-多普勒域相位 exp(j π fa² / Ka) 补偿。
 */
bool Compress2D(const ComplexMatrix& raw_pulse_history, const ComplexVector& range_matched_filter,
                const RangeAzimuthCompressionConfig& config, ComplexMatrix* output);

}  // namespace signal
}  // namespace sar

#endif  // ONEQ_SRC_SAR_SIGNAL_SAR_WAVEFORM_H_
