/**
 * @file SarOmegaKTruthEvaluationOrchestrator.h
 * @brief Identity-bound eligible truth evaluation orchestrator.
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_EVALUATION_ORCHESTRATOR_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_EVALUATION_ORCHESTRATOR_H_

#include <cstdint>
#include <vector>

#include "sar/imaging/SarOmegaKTruthEligibility.h"

namespace sar {
namespace imaging {

enum class OmegaKTruthEvaluationOrchestrationStatus { kEvaluated = 0, kRejected = 1 };
enum class OmegaKTruthEvaluationOrchestrationReason {
  kNone = 0,
  kInvalidRequestId = 1,
  kNotEligible = 2,
  kIngestionNotSuccessful = 3,
  kDatasetIdentityMismatch = 4,
  kEvaluationRejected = 5,
};

struct OmegaKTruthEvaluationOrchestrationRequest {
  std::uint64_t request_id{0U};
  OmegaKTruthEligibilityResult eligibility;
  OmegaKTruthIngestionResult ingestion;
  std::vector<double> absolute_slant_ranges_m;
  std::vector<double> azimuth_coordinates;
  signal::ComplexMatrix numerical_image_candidate;
};

struct OmegaKTruthEvaluationOrchestrationResult {
  std::uint64_t request_id{0U};
  OmegaKTruthEvaluationOrchestrationStatus status{
      OmegaKTruthEvaluationOrchestrationStatus::kRejected};
  OmegaKTruthEvaluationOrchestrationReason reason{
      OmegaKTruthEvaluationOrchestrationReason::kNone};
  std::string dataset_id;
  OmegaKPointTargetAcceptanceResult quality;
};

OmegaKTruthEvaluationOrchestrationResult OrchestrateOmegaKTruthEvaluation(
    const OmegaKTruthEvaluationOrchestrationRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_EVALUATION_ORCHESTRATOR_H_
