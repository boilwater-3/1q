/**
 * @file SarOmegaKRelativeDelayTransform.h
 * @brief Omega-K 收缩频谱相对延迟变换执行器。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_RELATIVE_DELAY_TRANSFORM_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_RELATIVE_DELAY_TRANSFORM_H_

#include <cstdint>
#include <vector>

#include "sar/imaging/SarOmegaKReducedRangeAxis.h"
#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

/**
 * @brief 相对延迟变换执行状态。
 */
enum class OmegaKRelativeDelayStatus { kSucceeded = 0, kRejected = 1 };
/**
 * @brief 相对延迟变换拒绝原因。
 */
enum class OmegaKRelativeDelayReason {
  kNone = 0,                 /**< 无 */
  kInvalidRequestId = 1,     /**< 请求 ID 非法 */
  kInvalidFrequencyAxis = 2, /**< 频率轴非法 */
  kInvalidSpectrum = 3,      /**< 频谱非法 */
  kTransformFailure = 4,      /**< 变换失败 */
};

/**
 * @brief 相对延迟变换请求。
 */
struct OmegaKRelativeDelayRequest {
  std::uint64_t request_id{0U};                       /**< 请求 ID */
  std::vector<double> reduced_range_frequencies_hz;   /**< 收缩距离频率轴（Hz） */
  signal::ComplexMatrix reduced_spectrum;             /**< 收缩后频谱 */
};

/**
 * @brief 相对延迟变换结果。
 */
struct OmegaKRelativeDelayResult {
  std::uint64_t request_id{0U};                       /**< 关联的请求 ID */
  OmegaKRelativeDelayStatus status{OmegaKRelativeDelayStatus::kRejected}; /**< 执行状态 */
  OmegaKRelativeDelayReason reason{OmegaKRelativeDelayReason::kNone}; /**< 拒绝原因 */
  OmegaKReducedRangeAxisDiagnostics axis_diagnostics; /**< 收缩轴诊断 */
  signal::ComplexMatrix relative_delay_domain;        /**< 相对延迟域矩阵 */
};

/**
 * @brief 对收缩频谱执行相对延迟变换（距离向 IFFT）。
 * @param[in] request 相对延迟变换请求。
 * @return 变换结果（含相对延迟域矩阵与轴诊断）。
 */
OmegaKRelativeDelayResult ExecuteOmegaKRelativeDelayTransform(
    const OmegaKRelativeDelayRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_RELATIVE_DELAY_TRANSFORM_H_
