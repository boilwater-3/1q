#include <gtest/gtest.h>

#include <cmath>

#include "sar/imaging/SarImageQuality.h"

namespace sar {
namespace {

signal::ComplexMatrix MakeZeroImage(std::size_t rows, std::size_t cols) {
  signal::ComplexMatrix image;
  image.rows = rows;
  image.cols = cols;
  image.values.assign(rows * cols, signal::ComplexSample(0.0, 0.0));
  return image;
}

TEST(SarImageQualityTest, MeasuresPeakMainlobeAndSidelobesDeterministically) {
  signal::ComplexMatrix image = MakeZeroImage(5U, 7U);
  image(2U, 3U) = signal::ComplexSample(10.0, 0.0);
  image(1U, 3U) = signal::ComplexSample(8.0, 0.0);
  image(3U, 3U) = signal::ComplexSample(8.0, 0.0);
  image(2U, 2U) = signal::ComplexSample(8.0, 0.0);
  image(2U, 4U) = signal::ComplexSample(8.0, 0.0);
  image(0U, 0U) = signal::ComplexSample(1.0, 0.0);

  const imaging::ImageQualityMetrics metrics = imaging::EvaluateImageQuality(image);

  ASSERT_TRUE(metrics.valid);
  EXPECT_EQ(metrics.peak_row, 2U);
  EXPECT_EQ(metrics.peak_col, 3U);
  EXPECT_DOUBLE_EQ(metrics.peak_magnitude, 10.0);
  EXPECT_DOUBLE_EQ(metrics.azimuth_width_3db_bins, 3.0);
  EXPECT_DOUBLE_EQ(metrics.range_width_3db_bins, 3.0);
  EXPECT_NEAR(metrics.pslr_db, -20.0, 1.0e-12);
  EXPECT_LT(metrics.islr_db, 0.0);
  EXPECT_GT(metrics.entropy_nats, 0.0);
}

TEST(SarImageQualityTest, RejectsEmptyAndZeroEnergyImages) {
  EXPECT_FALSE(imaging::EvaluateImageQuality(signal::ComplexMatrix{}).valid);
  EXPECT_FALSE(imaging::EvaluateImageQuality(MakeZeroImage(2U, 2U)).valid);
}

TEST(SarImageQualityTest, RemovesOnlyGlobalConstantPhaseForComparison) {
  signal::ComplexMatrix reference = MakeZeroImage(2U, 2U);
  reference.values = {signal::ComplexSample(1.0, 0.0), signal::ComplexSample(0.0, 2.0),
                      signal::ComplexSample(-3.0, 1.0), signal::ComplexSample(0.5, -0.25)};
  signal::ComplexMatrix candidate = reference;
  const double applied_phase = 0.75;
  const signal::ComplexSample rotation(std::cos(applied_phase), std::sin(applied_phase));
  for (signal::ComplexSample& sample : candidate.values) {
    sample *= rotation;
  }

  const imaging::ImageComparisonMetrics metrics =
      imaging::CompareImagesWithGlobalPhaseReference(reference, candidate);

  ASSERT_TRUE(metrics.valid);
  EXPECT_NEAR(metrics.phase_offset_rad, -applied_phase, 1.0e-12);
  EXPECT_NEAR(metrics.normalized_rms_error, 0.0, 1.0e-12);
  EXPECT_NEAR(metrics.coherent_correlation, 1.0, 1.0e-12);
}

TEST(SarImageQualityTest, DoesNotHideSpatiallyVaryingPhaseError) {
  signal::ComplexMatrix reference = MakeZeroImage(1U, 3U);
  reference.values.assign(3U, signal::ComplexSample(1.0, 0.0));
  signal::ComplexMatrix candidate = reference;
  candidate(0U, 1U) = signal::ComplexSample(0.0, 1.0);
  candidate(0U, 2U) = signal::ComplexSample(-1.0, 0.0);

  const imaging::ImageComparisonMetrics metrics =
      imaging::CompareImagesWithGlobalPhaseReference(reference, candidate);

  ASSERT_TRUE(metrics.valid);
  EXPECT_GT(metrics.normalized_rms_error, 0.5);
  EXPECT_LT(metrics.coherent_correlation, 0.5);
}

}  // namespace
}  // namespace sar
