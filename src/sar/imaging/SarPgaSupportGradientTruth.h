/**
 * @file SarPgaSupportGradientTruth.h
 * @brief 确定性 PGA 支撑选择与相位梯度真值。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_PGA_SUPPORT_GRADIENT_TRUTH_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_PGA_SUPPORT_GRADIENT_TRUTH_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

/**
 * @brief PGA 支撑梯度真值执行状态。
 */
enum class PgaSupportGradientTruthStatus { kSucceeded = 0, kRejected = 1 };
/**
 * @brief PGA 支撑梯度真值拒绝原因。
 */
enum class PgaSupportGradientTruthReason {
  kNone = 0,                 /**< 无 */
  kInvalidRequestId = 1,     /**< 请求 ID 非法 */
  kInvalidThreshold = 2,     /**< 门限非法 */
  kInvalidProfile = 3,       /**< 孔径剖面非法 */
  kZeroEnergy = 4,           /**< 剖面能量为零 */
  kInsufficientSupport = 5,  /**< 支撑样本不足 */
  kInvalidPhaseTruth = 6,     /**< 注入相位真值非法 */
};

/**
 * @brief PGA 支撑梯度真值请求。
 */
struct PgaSupportGradientTruthRequest {
  std::uint64_t request_id{0U};          /**< 请求 ID */
  signal::ComplexVector aperture_profile; /**< 孔径复剖面 */
  std::vector<double> injected_phase_rad; /**< 注入的相位真值（rad） */
  double peak_relative_threshold{0.0};    /**< 峰值相对门限（相对最大幅度） */
  std::size_t minimum_supported_samples{0U}; /**< 最小支撑样本数 */
};

/**
 * @brief PGA 支撑梯度真值结果。
 */
struct PgaSupportGradientTruthResult {
  std::uint64_t request_id{0U};          /**< 关联的请求 ID */
  PgaSupportGradientTruthStatus status{PgaSupportGradientTruthStatus::kRejected}; /**< 执行状态 */
  PgaSupportGradientTruthReason reason{PgaSupportGradientTruthReason::kNone}; /**< 拒绝原因 */
  std::size_t peak_index{0U};            /**< 峰值样本索引 */
  std::size_t supported_sample_count{0U}; /**< 支撑样本数 */
  std::vector<std::uint8_t> support_mask; /**< 支撑掩码 */
  std::vector<double> wrapped_forward_gradient_rad; /**< 缠绕前向梯度（rad） */
};

/**
 * @brief 由峰值相对门限确定性选择 PGA 支撑并计算相位梯度真值。
 * @param[in] request 支撑梯度真值请求。
 * @return 执行结果（含支撑掩码与缠绕梯度）。
 */
PgaSupportGradientTruthResult ExecutePgaSupportGradientTruth(
    const PgaSupportGradientTruthRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_PGA_SUPPORT_GRADIENT_TRUTH_H_
