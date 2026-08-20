/**
 * @file SarPgaGradientTruthComparison.h
 * @brief 有界 PGA 梯度估计的缠绕误差比较。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_PGA_GRADIENT_TRUTH_COMPARISON_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_PGA_GRADIENT_TRUTH_COMPARISON_H_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace sar {
namespace imaging {

/**
 * @brief PGA 梯度比较状态。
 */
enum class PgaGradientComparisonStatus { kPassed = 0, kFailed = 1, kRejected = 2 };
/**
 * @brief PGA 梯度比较拒绝原因。
 */
enum class PgaGradientComparisonReason {
  kNone = 0,                              /**< 无 */
  kInvalidRequestId = 1,                  /**< 请求 ID 非法 */
  kInvalidVectors = 2,                    /**< 梯度向量非法 */
  kInvalidTolerance = 3,                  /**< 容差非法 */
  kInsufficientJointlyValidPairs = 4,      /**< 联合有效样本对不足 */
};

/**
 * @brief PGA 梯度真值比较请求。
 */
struct PgaGradientComparisonRequest {
  std::uint64_t request_id{0U};                          /**< 请求 ID */
  std::vector<double> estimated_wrapped_gradient_rad;    /**< 估计的缠绕梯度（rad） */
  std::vector<std::uint8_t> estimator_valid_pair_mask;   /**< 估计侧有效对掩码 */
  std::vector<double> truth_wrapped_gradient_rad;        /**< 真值缠绕梯度（rad） */
  std::vector<std::uint8_t> truth_valid_pair_mask;       /**< 真值侧有效对掩码 */
  std::size_t minimum_jointly_valid_pair_count{0U};      /**< 最小联合有效对数 */
  double maximum_abs_wrapped_error_tolerance_rad{0.0};   /**< 最大缠绕误差绝对值容差（rad） */
  double rms_wrapped_error_tolerance_rad{0.0};           /**< RMS 缠绕误差容差（rad） */
};

/**
 * @brief PGA 梯度真值比较结果。
 */
struct PgaGradientComparisonResult {
  std::uint64_t request_id{0U};                          /**< 关联的请求 ID */
  PgaGradientComparisonStatus status{PgaGradientComparisonStatus::kRejected}; /**< 比较状态 */
  PgaGradientComparisonReason reason{PgaGradientComparisonReason::kNone}; /**< 拒绝原因 */
  std::size_t jointly_valid_pair_count{0U};              /**< 联合有效对数 */
  std::vector<std::uint8_t> jointly_valid_pair_mask;     /**< 联合有效对掩码 */
  double maximum_abs_wrapped_error_rad{0.0};             /**< 最大缠绕误差绝对值（rad） */
  double rms_wrapped_error_rad{0.0};                     /**< RMS 缠绕误差（rad） */
};

/**
 * @brief 比较估计 PGA 梯度与真值梯度的缠绕误差。
 * @param[in] request 比较请求。
 * @return 比较结果（含最大/RMS 缠绕误差与通过判定）。
 */
PgaGradientComparisonResult ComparePgaGradientTruth(
    const PgaGradientComparisonRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_PGA_GRADIENT_TRUTH_COMPARISON_H_
