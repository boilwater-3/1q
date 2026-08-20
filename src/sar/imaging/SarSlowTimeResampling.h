/**
 * @file SarSlowTimeResampling.h
 * @brief 时变 PRF 慢时间轴诊断与线性重采样。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_SLOW_TIME_RESAMPLING_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_SLOW_TIME_RESAMPLING_H_

#include <complex>
#include <cstddef>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

/**
 * @brief 慢时间线性重采样诊断。
 */
struct SlowTimeResamplingDiagnostics {
  bool valid{false};                              /**< 诊断是否有效 */
  bool uniform_within_tolerance{false};           /**< 实际间隔是否在容差内均匀 */
  std::size_t sample_count{0U};                   /**< 样本数 */
  double duration_s{0.0};                         /**< 时间轴跨度（s） */
  double nominal_interval_s{0.0};                 /**< 标称间隔（s） */
  double nominal_prf_hz{0.0};                     /**< 标称 PRF（Hz） */
  double minimum_actual_interval_s{0.0};          /**< 最小实际间隔（s） */
  double maximum_actual_interval_s{0.0};          /**< 最大实际间隔（s） */
  double maximum_abs_interval_deviation_s{0.0};   /**< 最大间隔偏差绝对值（s） */
  double interval_deviation_rms_s{0.0};           /**< 间隔偏差 RMS（s） */
  double maximum_abs_time_axis_deviation_s{0.0};  /**< 最大时间轴偏差绝对值（s） */
  double time_axis_deviation_rms_s{0.0};          /**< 时间轴偏差 RMS（s） */
  double uniform_tolerance_s{0.0};                /**< 均匀性容差（s） */
  std::vector<double> nominal_times_s;            /**< 重采样后的标称时间轴（s） */
};

/**
 * @brief 慢时间间隙（缺脉冲）诊断。
 */
struct SlowTimeGapDiagnostics {
  bool valid{false};                                        /**< 诊断是否有效 */
  bool resampling_allowed{false};                           /**< 是否允许重采样 */
  std::size_t sample_count{0U};                             /**< 样本数 */
  double expected_interval_s{0.0};                          /**< 期望间隔（s） */
  double expected_prf_hz{0.0};                              /**< 期望 PRF（Hz） */
  double minimum_actual_gap_s{0.0};                         /**< 最小实际间隙（s） */
  double maximum_actual_gap_s{0.0};                         /**< 最大实际间隙（s） */
  double maximum_gap_ratio{0.0};                            /**< 最大间隙比 */
  std::size_t rejected_gap_count{0U};                       /**< 被拒绝的间隙数 */
  std::size_t suspected_missing_pulse_count{0U};            /**< 疑似缺失脉冲数 */
  std::size_t first_rejected_gap_index{static_cast<std::size_t>(-1)}; /**< 首个被拒绝间隙索引 */
};

/**
 * @brief 诊断显式时间轴的慢时间间隙。
 * @param[in] explicit_times_s 显式慢时间轴（s）。
 * @param[in] expected_interval_s 期望均匀间隔（s）。
 * @param[out] diagnostics 间隙诊断。
 * @return 成功返回 true，失败返回 false。
 */
bool DiagnoseSlowTimeGaps(const std::vector<double>& explicit_times_s,
                          double expected_interval_s,
                          SlowTimeGapDiagnostics* diagnostics);

/**
 * @brief 按标称均匀间隔对一维复序列做慢时间线性重采样。
 * @param[in] explicit_times_s 显式慢时间轴（s）。
 * @param[in] input_samples 输入复样本。
 * @param[out] output_samples 重采样输出复样本。
 * @param[out] diagnostics 重采样诊断。
 * @return 成功返回 true，失败返回 false。
 */
bool ResampleSlowTimeLinear(const std::vector<double>& explicit_times_s,
                            const std::vector<std::complex<double>>& input_samples,
                            std::vector<std::complex<double>>* output_samples,
                            SlowTimeResamplingDiagnostics* diagnostics);

/**
 * @brief 对 raw history 逐列做慢时间线性重采样。
 * @param[in] explicit_times_s 显式慢时间轴（s）。
 * @param[in] input 输入 raw history。
 * @param[out] output 重采样后的 raw history。
 * @param[out] diagnostics 重采样诊断。
 * @return 成功返回 true，失败返回 false。
 */
bool ResampleRawHistorySlowTimeLinear(const std::vector<double>& explicit_times_s,
                                      const signal::ComplexMatrix& input,
                                      signal::ComplexMatrix* output,
                                      SlowTimeResamplingDiagnostics* diagnostics);

/**
 * @brief 带间隙保护的 raw history 慢时间线性重采样。
 *
 * 先做间隙诊断，仅当允许重采样时才执行线性重采样。
 * @param[in] explicit_times_s 显式慢时间轴（s）。
 * @param[in] expected_interval_s 期望均匀间隔（s）。
 * @param[in] input 输入 raw history。
 * @param[out] output 重采样后的 raw history。
 * @param[out] gap_diagnostics 间隙诊断。
 * @param[out] resampling_diagnostics 重采样诊断。
 * @return 间隙允许且重采样成功返回 true，否则返回 false。
 */
bool ResampleRawHistorySlowTimeLinearGuarded(
    const std::vector<double>& explicit_times_s, double expected_interval_s,
    const signal::ComplexMatrix& input, signal::ComplexMatrix* output,
    SlowTimeGapDiagnostics* gap_diagnostics,
    SlowTimeResamplingDiagnostics* resampling_diagnostics);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_SLOW_TIME_RESAMPLING_H_
