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
  EXPECT_FALSE(metrics.resolution_m_valid);
  EXPECT_NEAR(metrics.pslr_db, -20.0, 1.0e-12);
  EXPECT_LT(metrics.islr_db, 0.0);
  EXPECT_GT(metrics.entropy_nats, 0.0);
  EXPECT_GT(metrics.image_contrast, 0.0);
}

TEST(SarImageQualityTest, RejectsEmptyAndZeroEnergyImages) {
  EXPECT_FALSE(imaging::EvaluateImageQuality(signal::ComplexMatrix{}).valid);
  EXPECT_FALSE(imaging::EvaluateImageQuality(MakeZeroImage(2U, 2U)).valid);
}

TEST(SarImageQualityTest, ConvertsBinWidthsToMetersWhenSpacingIsValid) {
  signal::ComplexMatrix image = MakeZeroImage(3U, 5U);
  image(1U, 2U) = signal::ComplexSample(10.0, 0.0);
  image(1U, 1U) = signal::ComplexSample(8.0, 0.0);
  image(1U, 3U) = signal::ComplexSample(8.0, 0.0);

  imaging::ImageQualityConfig config;
  config.range_pixel_spacing_m = 1.5;
  config.azimuth_pixel_spacing_m = 2.0;
  const imaging::ImageQualityMetrics metrics = imaging::EvaluateImageQuality(image, config);

  ASSERT_TRUE(metrics.valid);
  EXPECT_TRUE(metrics.resolution_m_valid);
  EXPECT_DOUBLE_EQ(metrics.range_width_3db_bins, 3.0);
  EXPECT_DOUBLE_EQ(metrics.azimuth_width_3db_bins, 1.0);
  EXPECT_DOUBLE_EQ(metrics.range_resolution_3db_m, 4.5);
  EXPECT_DOUBLE_EQ(metrics.azimuth_resolution_3db_m, 2.0);
}

TEST(SarImageQualityTest, MainlobeMethodChangesDeterministicWidth) {
  signal::ComplexMatrix image = MakeZeroImage(1U, 7U);
  image(0U, 3U) = signal::ComplexSample(10.0, 0.0);
  image(0U, 2U) = signal::ComplexSample(3.0, 0.0);
  image(0U, 4U) = signal::ComplexSample(3.0, 0.0);
  image(0U, 1U) = signal::ComplexSample(2.0, 0.0);
  image(0U, 5U) = signal::ComplexSample(2.0, 0.0);

  imaging::ImageQualityConfig config;
  config.mainlobe_method = imaging::MainlobeEstimationMethod::k3dB;
  const imaging::ImageQualityMetrics three_db = imaging::EvaluateImageQuality(image, config);

  config.mainlobe_method = imaging::MainlobeEstimationMethod::k20dB;
  const imaging::ImageQualityMetrics twenty_db = imaging::EvaluateImageQuality(image, config);

  ASSERT_TRUE(three_db.valid);
  ASSERT_TRUE(twenty_db.valid);
  EXPECT_EQ(three_db.mainlobe_method, imaging::MainlobeEstimationMethod::k3dB);
  EXPECT_EQ(twenty_db.mainlobe_method, imaging::MainlobeEstimationMethod::k20dB);
  EXPECT_DOUBLE_EQ(three_db.range_width_3db_bins, 1.0);
  EXPECT_DOUBLE_EQ(twenty_db.range_width_3db_bins, 5.0);
}

TEST(SarImageQualityTest, UniformImageHasLowerContrastThanPointImage) {
  signal::ComplexMatrix uniform = MakeZeroImage(4U, 4U);
  uniform.values.assign(uniform.values.size(), signal::ComplexSample(1.0, 0.0));
  signal::ComplexMatrix point = MakeZeroImage(4U, 4U);
  point(2U, 2U) = signal::ComplexSample(10.0, 0.0);

  const imaging::ImageQualityMetrics uniform_metrics = imaging::EvaluateImageQuality(uniform);
  const imaging::ImageQualityMetrics point_metrics = imaging::EvaluateImageQuality(point);

  ASSERT_TRUE(uniform_metrics.valid);
  ASSERT_TRUE(point_metrics.valid);
  EXPECT_NEAR(uniform_metrics.image_contrast, 0.0, 1.0e-12);
  EXPECT_GT(point_metrics.image_contrast, uniform_metrics.image_contrast);
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

TEST(SarImageQualityTest, NormalizesGlobalAmplitudeScaleForShapeComparison) {
  signal::ComplexMatrix reference = MakeZeroImage(1U, 3U);
  reference.values = {signal::ComplexSample(1.0, 0.0), signal::ComplexSample(0.0, 2.0),
                      signal::ComplexSample(-1.0, 0.5)};
  signal::ComplexMatrix candidate = reference;
  for (signal::ComplexSample& sample : candidate.values) {
    sample *= 7.0;
  }

  const imaging::ImageComparisonMetrics metrics =
      imaging::CompareImagesWithGlobalPhaseReference(reference, candidate);

  ASSERT_TRUE(metrics.valid);
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
