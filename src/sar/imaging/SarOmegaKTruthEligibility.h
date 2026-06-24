/**
 * @file SarOmegaKTruthEligibility.h
 * @brief Eligibility gate for physical Omega-K truth evaluation.
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_ELIGIBILITY_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_ELIGIBILITY_H_

#include <cstdint>

#include "sar/imaging/SarOmegaKTruthIngestion.h"

namespace sar {
namespace imaging {

enum class OmegaKTruthEligibilityStatus { kEligible = 0, kIneligible = 1, kRejected = 2 };
enum class OmegaKTruthEligibilityReason {
  kNone = 0,
  kInvalidRequestId = 1,
  kIngestionNotSuccessful = 2,
  kNotPhysicalEvidence = 3,
  kNotIndependent = 4,
  kMissingProvenance = 5,
  kDigestNotVerified = 6,
};

struct OmegaKTruthEligibilityRequest {
  std::uint64_t request_id{0U};
  OmegaKTruthIngestionResult ingestion;
};

struct OmegaKTruthEligibilityResult {
  std::uint64_t request_id{0U};
  OmegaKTruthEligibilityStatus status{OmegaKTruthEligibilityStatus::kRejected};
  OmegaKTruthEligibilityReason reason{OmegaKTruthEligibilityReason::kNone};
  std::string dataset_id;
};

OmegaKTruthEligibilityResult EvaluateOmegaKTruthEligibility(
    const OmegaKTruthEligibilityRequest& request);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_OMEGA_K_TRUTH_ELIGIBILITY_H_
