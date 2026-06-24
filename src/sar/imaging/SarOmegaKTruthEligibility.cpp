#include "sar/imaging/SarOmegaKTruthEligibility.h"

#include <cctype>

namespace sar {
namespace imaging {

namespace {

OmegaKTruthEligibilityResult Make(const OmegaKTruthEligibilityRequest& request,
                                  OmegaKTruthEligibilityStatus status,
                                  OmegaKTruthEligibilityReason reason) {
  OmegaKTruthEligibilityResult result;
  result.request_id = request.request_id;
  result.status = status;
  result.reason = reason;
  return result;
}

bool IsSha256Hex(const std::string& value) {
  if (value.size() != 64U) {
    return false;
  }
  for (char character : value) {
    if (!std::isxdigit(static_cast<unsigned char>(character))) {
      return false;
    }
  }
  return true;
}

bool DigestsEqual(const std::string& lhs, const std::string& rhs) {
  if (!IsSha256Hex(lhs) || !IsSha256Hex(rhs)) {
    return false;
  }
  for (std::size_t index = 0U; index < lhs.size(); ++index) {
    if (std::tolower(static_cast<unsigned char>(lhs[index])) !=
        std::tolower(static_cast<unsigned char>(rhs[index]))) {
      return false;
    }
  }
  return true;
}

}  // namespace

OmegaKTruthEligibilityResult EvaluateOmegaKTruthEligibility(
    const OmegaKTruthEligibilityRequest& request) {
  if (request.request_id == 0U) {
    return Make(request, OmegaKTruthEligibilityStatus::kRejected,
                OmegaKTruthEligibilityReason::kInvalidRequestId);
  }
  if (request.ingestion.status != OmegaKTruthIngestionStatus::kSucceeded) {
    return Make(request, OmegaKTruthEligibilityStatus::kRejected,
                OmegaKTruthEligibilityReason::kIngestionNotSuccessful);
  }
  if (!request.ingestion.manifest.physical_evidence) {
    return Make(request, OmegaKTruthEligibilityStatus::kIneligible,
                OmegaKTruthEligibilityReason::kNotPhysicalEvidence);
  }
  if (!request.ingestion.manifest.truth.independently_generated) {
    return Make(request, OmegaKTruthEligibilityStatus::kIneligible,
                OmegaKTruthEligibilityReason::kNotIndependent);
  }
  if (request.ingestion.manifest.source.empty() ||
      request.ingestion.manifest.acquisition_date.empty()) {
    return Make(request, OmegaKTruthEligibilityStatus::kIneligible,
                OmegaKTruthEligibilityReason::kMissingProvenance);
  }
  if (!DigestsEqual(request.ingestion.computed_sha256,
                    request.ingestion.manifest.digest_sha256)) {
    return Make(request, OmegaKTruthEligibilityStatus::kIneligible,
                OmegaKTruthEligibilityReason::kDigestNotVerified);
  }
  OmegaKTruthEligibilityResult result =
      Make(request, OmegaKTruthEligibilityStatus::kEligible,
           OmegaKTruthEligibilityReason::kNone);
  result.dataset_id = request.ingestion.manifest.dataset_id;
  return result;
}

}  // namespace imaging
}  // namespace sar
