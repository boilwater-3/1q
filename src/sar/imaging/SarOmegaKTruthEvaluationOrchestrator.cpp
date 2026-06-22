// ============================================================================
// 【未进行设计需求，不再扩展 — DEPRECATED】
// 本文件不参与构建（见 src/sar/CMakeLists.txt 的 SAR_ENGINE_SOURCES 注释），
// 仅作为探索性参考保留。请勿新增依赖或据此实施。
// ============================================================================

﻿#include "sar/imaging/SarOmegaKTruthEvaluationOrchestrator.h"

namespace sar {
namespace imaging {

namespace {

OmegaKTruthEvaluationOrchestrationResult Reject(
    const OmegaKTruthEvaluationOrchestrationRequest& request,
    OmegaKTruthEvaluationOrchestrationReason reason) {
  OmegaKTruthEvaluationOrchestrationResult result;
  result.request_id = request.request_id;
  result.reason = reason;
  return result;
}

}  // namespace

OmegaKTruthEvaluationOrchestrationResult OrchestrateOmegaKTruthEvaluation(
    const OmegaKTruthEvaluationOrchestrationRequest& request) {
  if (request.request_id == 0U) {
    return Reject(request, OmegaKTruthEvaluationOrchestrationReason::kInvalidRequestId);
  }
  if (request.eligibility.status != OmegaKTruthEligibilityStatus::kEligible) {
    return Reject(request, OmegaKTruthEvaluationOrchestrationReason::kNotEligible);
  }
  if (request.ingestion.status != OmegaKTruthIngestionStatus::kSucceeded) {
    return Reject(request,
                  OmegaKTruthEvaluationOrchestrationReason::kIngestionNotSuccessful);
  }
  if (request.eligibility.dataset_id.empty() ||
      request.eligibility.dataset_id != request.ingestion.manifest.dataset_id) {
    return Reject(request,
                  OmegaKTruthEvaluationOrchestrationReason::kDatasetIdentityMismatch);
  }

  OmegaKPointTargetAcceptanceRequest evaluation;
  evaluation.request_id = request.request_id;
  evaluation.absolute_slant_ranges_m = request.absolute_slant_ranges_m;
  evaluation.azimuth_coordinates = request.azimuth_coordinates;
  evaluation.numerical_image_candidate = request.numerical_image_candidate;
  evaluation.truth = request.ingestion.manifest.truth;
  evaluation.tolerances = request.ingestion.manifest.tolerances;
  const OmegaKPointTargetAcceptanceResult quality =
      EvaluateOmegaKPointTargetCandidate(evaluation);
  if (quality.status == OmegaKPointTargetAcceptanceStatus::kRejected) {
    return Reject(request, OmegaKTruthEvaluationOrchestrationReason::kEvaluationRejected);
  }

  OmegaKTruthEvaluationOrchestrationResult result;
  result.request_id = request.request_id;
  result.status = OmegaKTruthEvaluationOrchestrationStatus::kEvaluated;
  result.reason = OmegaKTruthEvaluationOrchestrationReason::kNone;
  result.dataset_id = request.eligibility.dataset_id;
  result.quality = quality;
  return result;
}

}  // namespace imaging
}  // namespace sar
