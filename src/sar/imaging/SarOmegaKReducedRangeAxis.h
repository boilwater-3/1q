/**
 * @file SarOmegaKReducedRangeAxis.h
 * @brief Omega-K 收缩距离频率网格与相对延迟轴诊断。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_REDUCED_RANGE_AXIS_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_REDUCED_RANGE_AXIS_H_

#include <cstddef>
#include <vector>

namespace sar {
namespace imaging {

/**
 * @brief Omega-K 收缩距离频率网格与相对延迟轴诊断。
 */
struct OmegaKReducedRangeAxisDiagnostics {
  bool valid{false};                              /**< 诊断是否有效 */
  std::size_t sample_count{0U};                   /**< 收缩轴样本数 */
  double frequency_spacing_hz{0.0};               /**< 频率间隔（Hz） */
  double maximum_abs_spacing_deviation_hz{0.0};   /**< 最大间隔偏差绝对值（Hz） */
  double effective_bandwidth_hz{0.0};             /**< 有效带宽（Hz） */
  double unambiguous_delay_window_s{0.0};         /**< 无模糊延迟窗（s） */
  double relative_delay_spacing_s{0.0};           /**< 相对延迟间隔（s） */
  double relative_range_spacing_m{0.0};           /**< 相对距离间隔（m） */
  std::vector<double> relative_delays_s;          /**< 相对延迟轴（s） */
};

/**
 * @brief 诊断收缩距离频率网格并推导相对延迟轴。
 * @param[in] reduced_range_frequencies_hz 收缩后的距离频率轴（Hz）。
 * @param[out] diagnostics 收缩轴诊断。
 * @return 成功返回 true，失败返回 false。
 */
bool DiagnoseOmegaKReducedRangeAxis(
    const std::vector<double>& reduced_range_frequencies_hz,
    OmegaKReducedRangeAxisDiagnostics* diagnostics);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_REDUCED_RANGE_AXIS_H_
