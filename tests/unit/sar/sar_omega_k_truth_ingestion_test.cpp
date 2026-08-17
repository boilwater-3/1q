#include <gtest/gtest.h>

#include <string>

#include "sar/imaging/SarOmegaKTruthIngestion.h"

namespace sar {
namespace imaging {
namespace {

std::string ManifestWithDigest(const std::string& digest) {
  return
      "ONEQ_SAR_OMEGA_K_TRUTH_MANIFEST 1\n"
      "dataset_id synthetic-integrity-fixture\n"
      "schema_version 1\n"
      "physical_evidence false\n"
      "source repository-synthetic-test\n"
      "acquisition_date 2026-06-11\n"
      "independently_generated false\n"
      "inside_common_support true\n"
      "absolute_slant_range_m 1000\n"
      "azimuth_coordinate 0\n"
      "peak_phase_rad 0\n"
      "peak_magnitude 1\n"
      "range_mainlobe_half_width 0\n"
      "azimuth_mainlobe_half_width 0\n"
      "maximum_range_error_m 1\n"
      "maximum_azimuth_error 0.1\n"
      "maximum_abs_phase_error_rad 0.01\n"
      "maximum_relative_magnitude_error 0.02\n"
      "maximum_range_pslr_db -20\n"
      "maximum_azimuth_pslr_db -20\n"
      "maximum_range_islr_db -15\n"
      "maximum_azimuth_islr_db -15\n"
      "digest_sha256 " + digest + "\nEND\n";
}

OmegaKTruthIngestionRequest ValidRequest() {
  OmegaKTruthIngestionRequest request;
  request.request_id = 111U;
  request.payload_bytes = {'a', 'b', 'c'};
  request.manifest_text = ManifestWithDigest(
      "ba7816bf8f01cfea414140de5dae2223"
      "b00361a396177a9cb410ff61f20015ad");
  return request;
}

TEST(SarOmegaKTruthIngestionTest, PublishesOnlyAfterParseAndDigestMatch) {
  const OmegaKTruthIngestionResult result = IngestOmegaKTruth(ValidRequest());
  ASSERT_EQ(result.status, OmegaKTruthIngestionStatus::kSucceeded);
  EXPECT_FALSE(result.manifest.physical_evidence);
  EXPECT_FALSE(result.manifest.truth.independently_generated);
  EXPECT_EQ(result.payload_bytes, ValidRequest().payload_bytes);
  EXPECT_EQ(result.computed_sha256, result.manifest.digest_sha256);
}

TEST(SarOmegaKTruthIngestionTest, RejectsMismatchAtomically) {
  OmegaKTruthIngestionRequest request = ValidRequest();
  request.payload_bytes.push_back('d');
  const OmegaKTruthIngestionResult result = IngestOmegaKTruth(request);
  EXPECT_EQ(result.reason, OmegaKTruthIngestionReason::kDigestMismatch);
  EXPECT_TRUE(result.payload_bytes.empty());
  EXPECT_TRUE(result.manifest.dataset_id.empty());
  EXPECT_TRUE(result.computed_sha256.empty());
}

TEST(SarOmegaKTruthIngestionTest, ReportsManifestFailureSeparately) {
  OmegaKTruthIngestionRequest request = ValidRequest();
  request.manifest_text = "invalid";
  EXPECT_EQ(IngestOmegaKTruth(request).reason,
            OmegaKTruthIngestionReason::kManifestRejected);
}

}  // namespace
}  // namespace imaging
}  // namespace sar
