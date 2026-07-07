/**
 * @file SarMotionCompensation.h
 * @brief SAR 内部一阶运动补偿工具。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_MOTION_COMPENSATION_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_MOTION_COMPENSATION_H_

#include <cstddef>
#include <vector>

#include "sar/geometry/SarGeometry.h"
#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

/**
 * @brief 一阶运动补偿配置。
 */
struct FirstOrderMotionCompensationConfig {
  double sample_rate_hz{0.0};        /**< 采样率（Hz） */
  double carrier_frequency_hz{0.0};  /**< 载频（Hz） */
  geometry::LocalPoint reference_point_m{}; /**< 补偿参考点（m） */
};

/**
 * @brief 一阶运动补偿诊断信息。
 */
struct MotionCompensationDiagnostics {
  double max_abs_range_error_m{0.0};       /**< 最大斜距误差绝对值（m） */
  double rms_range_error_m{0.0};           /**< RMS 斜距误差（m） */
  double max_abs_envelope_shift_bins{0.0}; /**< 最大包络位移绝对值（bin） */
  std::size_t compensated_pulses{0U};      /**< 已补偿脉冲数 */
  std::size_t out_of_bounds_samples{0U};   /**< 越界样本数 */
};

/**
 * @brief 对实际轨迹 raw history 执行一阶运动补偿。
 *
 * 以理想轨迹为参考，按实际与理想斜距差对实际 raw history 逐脉冲做包络位移与相位校正。
 * @param[in] config 补偿配置。
 * @param[in] ideal_trajectory 理想轨迹脉冲序列。
 * @param[in] actual_trajectory 实际轨迹脉冲序列。
 * @param[in] actual_raw_pulse_history 实际轨迹原始相位历史。
 * @param[out] compensated 补偿后的相位历史。
 * @param[out] diagnostics 补偿诊断。
 * @return 成功返回 true，失败返回 false。
 */
bool ApplyFirstOrderMotionCompensation(
    const FirstOrderMotionCompensationConfig& config,
    const std::vector<geometry::PlatformPulseState>& ideal_trajectory,
    const std::vector<geometry::PlatformPulseState>& actual_trajectory,
    const signal::ComplexMatrix& actual_raw_pulse_history, signal::ComplexMatrix* compensated,
    MotionCompensationDiagnostics* diagnostics);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_MOTION_COMPENSATION_H_
