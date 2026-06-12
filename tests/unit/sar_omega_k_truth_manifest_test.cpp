#include <gtest/gtest.h>

#include <string>

#include "sar/imaging/SarOmegaKTruthManifest.h"

namespace sar {
namespace imaging {
namespace {

std::string ValidSyntheticManifest() {
  return
      "ONEQ_SAR_OMEGA_K_TRUTH_MANIFEST 1\n"
      "dataset_id synthetic-fixture-v1\n"
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
      "digest_sha256 "
      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef\n"
      "END\n";
}

TEST(SarOmegaKTruthManifestTest, ParsesStrictVersionedSyntheticFixture) {
  const OmegaKTruthManifestParseResult result =
      ParseOmegaKTruthManifest(ValidSyntheticManifest());
  ASSERT_EQ(result.status, OmegaKTruthManifestStatus::kParsed);
  EXPECT_EQ(result.manifest.dataset_id, "synthetic-fixture-v1");
  EXPECT_FALSE(result.manifest.physical_evidence);
  EXPECT_FALSE(result.manifest.truth.independently_generated);
  EXPECT_DOUBLE_EQ(result.manifest.truth.absolute_slant_range_m, 1000.0);
}

TEST(SarOmegaKTruthManifestTest, RejectsUnsupportedVersionAndInvalidDigest) {
  std::string manifest = ValidSyntheticManifest();
  manifest.replace(manifest.find("MANIFEST 1"), 10U, "MANIFEST 2");
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kUnsupportedVersion);

  manifest = ValidSyntheticManifest();
  manifest.replace(manifest.find("0123456789abcdef"), 64U, "not-a-digest");
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kInvalidDigest);
}

TEST(SarOmegaKTruthManifestTest, RejectsReorderedOrTrailingContentAtomically) {
  std::string manifest = ValidSyntheticManifest();
  manifest.replace(manifest.find("dataset_id"), 10U, "wrong_key_");
  const OmegaKTruthManifestParseResult wrong_key = ParseOmegaKTruthManifest(manifest);
  EXPECT_EQ(wrong_key.reason, OmegaKTruthManifestReason::kInvalidField);
  EXPECT_TRUE(wrong_key.manifest.dataset_id.empty());

  manifest = ValidSyntheticManifest() + "unexpected\n";
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kUnexpectedContent);
}

}  // namespace
}  // namespace imaging
}  // namespace sar
