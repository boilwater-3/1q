/**
 * @file SarPgaPhaseGradientEstimator.h
 * @brief 有界邻样本 PGA 相位梯度估计器。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_PGA_PHASE_GRADIENT_ESTIMATOR_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_PGA_PHASE_GRADIENT_ESTIMATOR_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

/**
 * @brief PGA 相位梯度估计状态。
 */
enum class PgaPhaseGradientEstimatorStatus { kSucceeded = 0, kRejected = 1 };
/**
 * @brief PGA 相位梯度估计拒绝原因。
 */
enum class PgaPhaseGradientEstimatorReason {
  kNone = 0,                    /**< 无 */
  kInvalidRequestId = 1,        /**< 请求 ID 非法 */
  kInvalidProfile = 2,          /**< 孔径剖面非法 */
  kInvalidSupportMask = 3,      /**< 支撑掩码非法 */
  kInsufficientValidPairs = 4,   /**< 有效样本对不足 */
};

/**
 * @brief PGA 相位梯度估计请求。
 */
struct PgaPhaseGradientEstimatorRequest {
  std::uint64_t request_id{0U};          /**< 请求 ID */
  signal::ComplexVector aperture_profile; /**< 孔径复剖面 */
  std::vector<std::uint8_t> support_mask; /**< 支撑（高信噪比）样本掩码 */
  std::size_t minimum_valid_pair_count{0U}; /**< 最小有效样本对数 */
};

/**
 * @brief PGA 相位梯度估计结果。
 */
struct PgaPhaseGradientEstimatorResult {
  std::uint64_t request_id{0U};          /**< 关联的请求 ID */
  PgaPhaseGradientEstimatorStatus status{PgaPhaseGradientEstimatorStatus::kRejected}; /**< 执行状态 */
  PgaPhaseGradientEstimatorReason reason{PgaPhaseGradientEstimatorReason::kNone}; /**< 拒绝原因 */
  std::size_t valid_pair_count{0U};      /**< 有效样本对数 */
  std::vector<std::uint8_t> valid_pair_mask; /**< 有效样本对掩码 */
  std::vector<double> wrapped_gradient_rad; /**< 缠绕相位梯度（rad） */
};

/**
 * @brief 由支撑掩码内的相邻样本估计 PGA 缠绕相位梯度。
 * @param[in] request 估计请求。
 * @return 估计结果（含缠绕梯度与有效对掩码）。
 */
PgaPhaseGradientEstimatorResult EstimatePgaPhaseGradient(
    const PgaPhaseGradientEstimatorRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_PGA_PHASE_GRADIENT_ESTIMATOR_H_
