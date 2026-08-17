/**
 * @file SarOmegaKTruthEvaluationOrchestrator.h
 * @brief 身份绑定的合格真值评估编排器。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_EVALUATION_ORCHESTRATOR_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_EVALUATION_ORCHESTRATOR_H_

#include <cstdint>
#include <vector>

#include "sar/imaging/SarOmegaKTruthEligibility.h"

namespace sar {
namespace imaging {

/**
 * @brief 真值评估编排状态。
 */
enum class OmegaKTruthEvaluationOrchestrationStatus { kEvaluated = 0, kRejected = 1 };
/**
 * @brief 真值评估编排拒绝原因。
 */
enum class OmegaKTruthEvaluationOrchestrationReason {
  kNone = 0,                    /**< 无 */
  kInvalidRequestId = 1,        /**< 请求 ID 非法 */
  kNotEligible = 2,             /**< 未通过准入 */
  kIngestionNotSuccessful = 3,  /**< 摄入未成功 */
  kDatasetIdentityMismatch = 4, /**< 数据集身份不一致 */
  kEvaluationRejected = 5,       /**< 点目标评估被拒绝 */
};

/**
 * @brief 真值评估编排请求。
 */
struct OmegaKTruthEvaluationOrchestrationRequest {
  std::uint64_t request_id{0U};               /**< 请求 ID */
  OmegaKTruthEligibilityResult eligibility;    /**< 准入结果 */
  OmegaKTruthIngestionResult ingestion;        /**< 摄入结果 */
  std::vector<double> absolute_slant_ranges_m; /**< 绝对斜距轴（m） */
  std::vector<double> azimuth_coordinates;    /**< 方位坐标 */
  signal::ComplexMatrix numerical_image_candidate; /**< 数值图像候选 */
};

/**
 * @brief 真值评估编排结果。
 */
struct OmegaKTruthEvaluationOrchestrationResult {
  std::uint64_t request_id{0U};               /**< 关联的请求 ID */
  OmegaKTruthEvaluationOrchestrationStatus status{
      OmegaKTruthEvaluationOrchestrationStatus::kRejected}; /**< 编排状态 */
  OmegaKTruthEvaluationOrchestrationReason reason{
      OmegaKTruthEvaluationOrchestrationReason::kNone}; /**< 拒绝原因 */
  std::string dataset_id;                    /**< 数据集 ID */
  OmegaKPointTargetAcceptanceResult quality;  /**< 点目标验收质量结果 */
};

/**
 * @brief 编排合格真值的点目标评估。
 *
 * 校验数据集身份一致后调用点目标验收，返回编排结果。
 * @param[in] request 编排请求。
 * @return 编排结果（含数据集 ID 与质量结果）。
 */
OmegaKTruthEvaluationOrchestrationResult OrchestrateOmegaKTruthEvaluation(
    const OmegaKTruthEvaluationOrchestrationRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_EVALUATION_ORCHESTRATOR_H_
