/**
 * @file SarOmegaKTruthEligibility.h
 * @brief 物理证据 Omega-K 真值评估的准入门。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_ELIGIBILITY_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_ELIGIBILITY_H_

#include <cstdint>

#include "sar/imaging/SarOmegaKTruthIngestion.h"

namespace sar {
namespace imaging {

/**
 * @brief 真值准入状态。
 */
enum class OmegaKTruthEligibilityStatus { kEligible = 0, kIneligible = 1, kRejected = 2 };
/**
 * @brief 真值准入拒绝/不符原因。
 */
enum class OmegaKTruthEligibilityReason {
  kNone = 0,                 /**< 无 */
  kInvalidRequestId = 1,     /**< 请求 ID 非法 */
  kIngestionNotSuccessful = 2, /**< 摄入未成功 */
  kNotPhysicalEvidence = 3,  /**< 非物理证据 */
  kNotIndependent = 4,       /**< 非独立来源 */
  kMissingProvenance = 5,    /**< 缺少溯源信息 */
  kDigestNotVerified = 6,     /**< 载荷摘要未通过校验 */
};

/**
 * @brief 真值准入请求。
 */
struct OmegaKTruthEligibilityRequest {
  std::uint64_t request_id{0U};   /**< 请求 ID */
  OmegaKTruthIngestionResult ingestion; /**< 摄入结果 */
};

/**
 * @brief 真值准入结果。
 */
struct OmegaKTruthEligibilityResult {
  std::uint64_t request_id{0U};   /**< 关联的请求 ID */
  OmegaKTruthEligibilityStatus status{OmegaKTruthEligibilityStatus::kRejected}; /**< 准入状态 */
  OmegaKTruthEligibilityReason reason{OmegaKTruthEligibilityReason::kNone}; /**< 拒绝原因 */
  std::string dataset_id;        /**< 数据集 ID */
};

/**
 * @brief 评估数据集是否满足物理证据真值准入条件。
 * @param[in] request 准入请求。
 * @return 准入结果（含状态与数据集 ID）。
 */
OmegaKTruthEligibilityResult EvaluateOmegaKTruthEligibility(
    const OmegaKTruthEligibilityRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_ELIGIBILITY_H_
