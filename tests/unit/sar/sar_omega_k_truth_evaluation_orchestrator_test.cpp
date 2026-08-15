#include <gtest/gtest.h>

#include "sar/imaging/SarOmegaKTruthEvaluationOrchestrator.h"

namespace sar {
namespace imaging {
namespace {

OmegaKTruthEvaluationOrchestrationRequest ValidRequest() {
  OmegaKTruthEvaluationOrchestrationRequest request;
  request.request_id = 131U;
  request.eligibility.status = OmegaKTruthEligibilityStatus::kEligible;
  request.eligibility.dataset_id = "external-point-target-v1";
  request.ingestion.status = OmegaKTruthIngestionStatus::kSucceeded;
  request.ingestion.manifest.dataset_id = request.eligibility.dataset_id;
  request.ingestion.manifest.truth.independently_generated = true;
  request.ingestion.manifest.truth.inside_common_support = true;
  request.ingestion.manifest.truth.absolute_slant_range_m = 1000.0;
  request.ingestion.manifest.truth.azimuth_coordinate = 0.0;
  request.ingestion.manifest.truth.peak_phase_rad = 0.0;
  request.ingestion.manifest.truth.peak_magnitude = 1.0;
  request.ingestion.manifest.tolerances.maximum_range_error_m = 0.0;
  request.ingestion.manifest.tolerances.maximum_azimuth_error = 0.0;
  request.ingestion.manifest.tolerances.maximum_abs_phase_error_rad = 0.0;
  request.ingestion.manifest.tolerances.maximum_relative_magnitude_error = 0.0;
  request.ingestion.manifest.tolerances.maximum_range_pslr_db = -10.0;
  request.ingestion.manifest.tolerances.maximum_azimuth_pslr_db = -10.0;
  request.ingestion.manifest.tolerances.maximum_range_islr_db = -10.0;
  request.ingestion.manifest.tolerances.maximum_azimuth_islr_db = -10.0;
  request.absolute_slant_ranges_m = {990.0, 1000.0, 1010.0};
  request.azimuth_coordinates = {-1.0, 0.0, 1.0};
  request.numerical_image_candidate.rows = 3U;
  request.numerical_image_candidate.cols = 3U;
  request.numerical_image_candidate.values = {
      {0.01, 0.0}, {0.01, 0.0}, {0.01, 0.0},
      {0.01, 0.0}, {1.0, 0.0}, {0.01, 0.0},
      {0.01, 0.0}, {0.01, 0.0}, {0.01, 0.0},
  };
  return request;
}

TEST(SarOmegaKTruthEvaluationOrchestratorTest, EvaluatesIdentityBoundEligibleTruth) {
  const OmegaKTruthEvaluationOrchestrationResult result =
      OrchestrateOmegaKTruthEvaluation(ValidRequest());
  ASSERT_EQ(result.status, OmegaKTruthEvaluationOrchestrationStatus::kEvaluated);
  EXPECT_EQ(result.dataset_id, "external-point-target-v1");
  EXPECT_EQ(result.quality.status, OmegaKPointTargetAcceptanceStatus::kPassed);
}

TEST(SarOmegaKTruthEvaluationOrchestratorTest, PreservesQualityFailureAsEvaluation) {
  OmegaKTruthEvaluationOrchestrationRequest request = ValidRequest();
  request.ingestion.manifest.truth.absolute_slant_range_m = 1001.0;
  const OmegaKTruthEvaluationOrchestrationResult result =
      OrchestrateOmegaKTruthEvaluation(request);
  EXPECT_EQ(result.status, OmegaKTruthEvaluationOrchestrationStatus::kEvaluated);
  EXPECT_EQ(result.quality.status, OmegaKPointTargetAcceptanceStatus::kFailed);
}

TEST(SarOmegaKTruthEvaluationOrchestratorTest, RejectsIneligibleOrMismatchedIdentity) {
  OmegaKTruthEvaluationOrchestrationRequest request = ValidRequest();
  request.eligibility.status = OmegaKTruthEligibilityStatus::kIneligible;
  EXPECT_EQ(OrchestrateOmegaKTruthEvaluation(request).reason,
            OmegaKTruthEvaluationOrchestrationReason::kNotEligible);

  request = ValidRequest();
  request.ingestion.manifest.dataset_id = "different-dataset";
  const OmegaKTruthEvaluationOrchestrationResult result =
      OrchestrateOmegaKTruthEvaluation(request);
  EXPECT_EQ(result.reason,
            OmegaKTruthEvaluationOrchestrationReason::kDatasetIdentityMismatch);
  EXPECT_TRUE(result.dataset_id.empty());
}

}  // namespace
}  // namespace imaging
}  // namespace sar
