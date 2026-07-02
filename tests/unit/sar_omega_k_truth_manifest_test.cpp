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

// =============================================================================
// schema_version / ReadBool 失败路径
// =============================================================================

TEST(SarOmegaKTruthManifestTest, RejectsUnsupportedSchemaVersion) {
  std::string manifest = ValidSyntheticManifest();
  // schema_version 1 -> schema_version 2
  manifest.replace(manifest.find("schema_version 1"), 17U, "schema_version 2");
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kUnsupportedVersion);
}

TEST(SarOmegaKTruthManifestTest, RejectsInvalidBoolValueForPhysicalEvidence) {
  std::string manifest = ValidSyntheticManifest();
  // physical_evidence false -> physical_evidence maybe
  manifest.replace(manifest.find("false\nsource"), 5U, "maybe");
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kInvalidField);
}

TEST(SarOmegaKTruthManifestTest, RejectsInvalidBoolValueForIndependentlyGenerated) {
  std::string manifest = ValidSyntheticManifest();
  // independently_generated false -> independently_generated 1
  manifest.replace(manifest.find("false\ninside"), 5U, "1");
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kInvalidField);
}

// =============================================================================
// 数值有限性 / 非负校验（逐字段触发 IsFiniteNonnegative）
// =============================================================================

TEST(SarOmegaKTruthManifestTest, RejectsNanSlantRange) {
  std::string manifest = ValidSyntheticManifest();
  manifest.replace(manifest.find("absolute_slant_range_m 1000"), 25U,
                   "absolute_slant_range_m nan");
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kInvalidField);
}

TEST(SarOmegaKTruthManifestTest, RejectsNegativeSlantRange) {
  std::string manifest = ValidSyntheticManifest();
  manifest.replace(manifest.find("absolute_slant_range_m 1000"), 25U,
                   "absolute_slant_range_m -1");
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kInvalidField);
}

TEST(SarOmegaKTruthManifestTest, RejectsNanAzimuthCoordinate) {
  std::string manifest = ValidSyntheticManifest();
  manifest.replace(manifest.find("azimuth_coordinate 0"), 21U,
                   "azimuth_coordinate nan");
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kInvalidField);
}

TEST(SarOmegaKTruthManifestTest, RejectsNanPeakPhase) {
  std::string manifest = ValidSyntheticManifest();
  manifest.replace(manifest.find("peak_phase_rad 0"), 16U, "peak_phase_rad nan");
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kInvalidField);
}

TEST(SarOmegaKTruthManifestTest, RejectsNanPeakMagnitude) {
  std::string manifest = ValidSyntheticManifest();
  manifest.replace(manifest.find("peak_magnitude 1"), 16U, "peak_magnitude nan");
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kInvalidField);
}

TEST(SarOmegaKTruthManifestTest, RejectsZeroPeakMagnitude) {
  std::string manifest = ValidSyntheticManifest();
  manifest.replace(manifest.find("peak_magnitude 1"), 16U, "peak_magnitude 0");
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kInvalidField);
}

TEST(SarOmegaKTruthManifestTest, RejectsNanRangeErrorTolerance) {
  std::string manifest = ValidSyntheticManifest();
  manifest.replace(manifest.find("maximum_range_error_m 1"), 23U,
                   "maximum_range_error_m nan");
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kInvalidField);
}

TEST(SarOmegaKTruthManifestTest, RejectsNegativeAzimuthErrorTolerance) {
  std::string manifest = ValidSyntheticManifest();
  manifest.replace(manifest.find("maximum_azimuth_error 0.1"), 25U,
                   "maximum_azimuth_error -0.1");
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kInvalidField);
}

TEST(SarOmegaKTruthManifestTest, RejectsNanPslrTolerance) {
  std::string manifest = ValidSyntheticManifest();
  manifest.replace(manifest.find("maximum_range_pslr_db -20"), 25U,
                   "maximum_range_pslr_db nan");
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kInvalidField);
}

TEST(SarOmegaKTruthManifestTest, RejectsNanIslrTolerance) {
  std::string manifest = ValidSyntheticManifest();
  manifest.replace(manifest.find("maximum_range_islr_db -15"), 25U,
                   "maximum_range_islr_db nan");
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kInvalidField);
}

// =============================================================================
// digest / END 标记的边界路径
// =============================================================================

TEST(SarOmegaKTruthManifestTest, RejectsDigestWithNonHexChars) {
  std::string manifest = ValidSyntheticManifest();
  // 替换 64 字符 digest 为含非 hex 字符的 64 字符串
  manifest.replace(manifest.find("0123456789abcdef"), 64U,
                   "zz23456789abcdef0123456789abcdef0123456789abcdef0123456789abczz");
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kInvalidDigest);
}

TEST(SarOmegaKTruthManifestTest, RejectsMutatedEndToken) {
  std::string manifest = ValidSyntheticManifest();
  // END -> DONE
  manifest.replace(manifest.find("END\n"), 3U, "DONE");
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kUnexpectedContent);
}

TEST(SarOmegaKTruthManifestTest, RejectsTruncatedManifestMissingEnd) {
  std::string manifest = ValidSyntheticManifest();
  manifest.erase(manifest.find("END\n"));
  EXPECT_EQ(ParseOmegaKTruthManifest(manifest).reason,
            OmegaKTruthManifestReason::kUnexpectedContent);
}

}  // namespace
}  // namespace imaging
}  // namespace sar
