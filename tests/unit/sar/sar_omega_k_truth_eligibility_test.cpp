#include <gtest/gtest.h>

#include "sar/imaging/SarOmegaKTruthEligibility.h"

namespace sar {
namespace imaging {
namespace {

OmegaKTruthEligibilityRequest EligibleRequest() {
  OmegaKTruthEligibilityRequest request;
  request.request_id = 121U;
  request.ingestion.status = OmegaKTruthIngestionStatus::kSucceeded;
  request.ingestion.manifest.dataset_id = "external-point-target-v1";
  request.ingestion.manifest.physical_evidence = true;
  request.ingestion.manifest.truth.independently_generated = true;
  request.ingestion.manifest.source = "external-reference-system";
  request.ingestion.manifest.acquisition_date = "2026-06-11";
  request.ingestion.manifest.digest_sha256 =
      "ba7816bf8f01cfea414140de5dae2223"
      "b00361a396177a9cb410ff61f20015ad";
  request.ingestion.computed_sha256 = request.ingestion.manifest.digest_sha256;
  return request;
}

TEST(SarOmegaKTruthEligibilityTest, AuthorizesEligibleDatasetForEvaluationOnly) {
  const OmegaKTruthEligibilityResult result =
      EvaluateOmegaKTruthEligibility(EligibleRequest());
  EXPECT_EQ(result.status, OmegaKTruthEligibilityStatus::kEligible);
  EXPECT_EQ(result.reason, OmegaKTruthEligibilityReason::kNone);
  EXPECT_EQ(result.dataset_id, "external-point-target-v1");
}

TEST(SarOmegaKTruthEligibilityTest, AcceptsVerifiedUppercaseManifestDigest) {
  OmegaKTruthEligibilityRequest request = EligibleRequest();
  request.ingestion.manifest.digest_sha256 =
      "BA7816BF8F01CFEA414140DE5DAE2223"
      "B00361A396177A9CB410FF61F20015AD";
  EXPECT_EQ(EvaluateOmegaKTruthEligibility(request).status,
            OmegaKTruthEligibilityStatus::kEligible);
}

TEST(SarOmegaKTruthEligibilityTest, KeepsSyntheticFixtureIneligible) {
  OmegaKTruthEligibilityRequest request = EligibleRequest();
  request.ingestion.manifest.physical_evidence = false;
  const OmegaKTruthEligibilityResult result = EvaluateOmegaKTruthEligibility(request);
  EXPECT_EQ(result.status, OmegaKTruthEligibilityStatus::kIneligible);
  EXPECT_EQ(result.reason, OmegaKTruthEligibilityReason::kNotPhysicalEvidence);
  EXPECT_TRUE(result.dataset_id.empty());
}

TEST(SarOmegaKTruthEligibilityTest, RejectsFailedIngestionAndUnverifiedDigest) {
  OmegaKTruthEligibilityRequest request = EligibleRequest();
  request.ingestion.status = OmegaKTruthIngestionStatus::kRejected;
  EXPECT_EQ(EvaluateOmegaKTruthEligibility(request).reason,
            OmegaKTruthEligibilityReason::kIngestionNotSuccessful);

  request = EligibleRequest();
  request.ingestion.computed_sha256[0] = '0';
  EXPECT_EQ(EvaluateOmegaKTruthEligibility(request).reason,
            OmegaKTruthEligibilityReason::kDigestNotVerified);
}

}  // namespace
}  // namespace imaging
}  // namespace sar
