#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "sar/calibration/SarRadiometricCalibration.h"

namespace sar {
namespace calibration {
namespace {

CalibrationSample MakeSample(double rcs_m2, double slant_range_m, double system_gain,
                             double weight = 1.0) {
  CalibrationSample sample;
  sample.known_rcs_m2 = rcs_m2;
  sample.slant_range_m = slant_range_m;
  sample.image_power = system_gain * rcs_m2 / std::pow(slant_range_m, 4.0);
  sample.weight = weight;
  return sample;
}

TEST(SarRadiometricCalibrationTest, SingleTargetClosesRcsLoop) {
  const CalibrationSample sample = MakeSample(9.0, 30.0, 4.0);
  RadiometricCalibration calibration;
  ASSERT_TRUE(CalibrateSingle(sample, &calibration));
  EXPECT_TRUE(calibration.valid);
  EXPECT_NEAR(calibration.image_calibration_factor, 0.25, 1.0e-15);
  ASSERT_EQ(calibration.residual_error_db.size(), 1U);
  EXPECT_NEAR(calibration.residual_error_db[0], 0.0, 1.0e-12);

  double measured_rcs_m2 = 0.0;
  ASSERT_TRUE(InvertRcs(sample.image_power, sample.slant_range_m, calibration, &measured_rcs_m2));
  EXPECT_NEAR(measured_rcs_m2, sample.known_rcs_m2, 1.0e-12);
}

TEST(SarRadiometricCalibrationTest, AmplitudeAndRangeScalingPreserveContract) {
  const CalibrationSample sample = MakeSample(4.0, 20.0, 5.0);
  RadiometricCalibration calibration;
  ASSERT_TRUE(CalibrateSingle(sample, &calibration));

  double scaled_rcs_m2 = 0.0;
  ASSERT_TRUE(InvertRcs(4.0 * sample.image_power, sample.slant_range_m, calibration,
                        &scaled_rcs_m2));
  EXPECT_NEAR(scaled_rcs_m2, 4.0 * sample.known_rcs_m2, 1.0e-12);

  const CalibrationSample farther = MakeSample(4.0, 40.0, 5.0);
  double farther_rcs_m2 = 0.0;
  ASSERT_TRUE(
      InvertRcs(farther.image_power, farther.slant_range_m, calibration, &farther_rcs_m2));
  EXPECT_NEAR(farther_rcs_m2, sample.known_rcs_m2, 1.0e-12);
}

TEST(SarRadiometricCalibrationTest, MultiTargetFusionUsesExplicitWeights) {
  const std::vector<CalibrationSample> consistent = {MakeSample(1.0, 10.0, 4.0, 1.0),
                                                     MakeSample(9.0, 30.0, 4.0, 3.0)};
  RadiometricCalibration calibration;
  ASSERT_TRUE(CalibrateMultiple(consistent, &calibration));
  EXPECT_NEAR(calibration.image_calibration_factor, 0.25, 1.0e-15);
  EXPECT_NEAR(calibration.weight_sum, 4.0, 1.0e-15);
  ASSERT_EQ(calibration.residual_error_db.size(), 2U);
  EXPECT_NEAR(calibration.residual_error_db[0], 0.0, 1.0e-12);
  EXPECT_NEAR(calibration.residual_error_db[1], 0.0, 1.0e-12);

  std::vector<CalibrationSample> perturbed = consistent;
  perturbed[0].image_power *= 2.0;
  ASSERT_TRUE(CalibrateMultiple(perturbed, &calibration));
  EXPECT_NEAR(calibration.image_calibration_factor, 0.21875, 1.0e-15);
  EXPECT_GT(calibration.residual_error_db[0], 0.0);
  EXPECT_GT(calibration.residual_error_db[1], -1.0);
}

TEST(SarRadiometricCalibrationTest, InvalidInputsAreRejected) {
  CalibrationSample sample = MakeSample(1.0, 10.0, 4.0);
  RadiometricCalibration calibration;
  sample.image_power = 0.0;
  EXPECT_FALSE(CalibrateSingle(sample, &calibration));
  sample = MakeSample(1.0, 10.0, 4.0);
  sample.weight = -1.0;
  EXPECT_FALSE(CalibrateSingle(sample, &calibration));
  sample = MakeSample(1.0, 10.0, 4.0);
  sample.known_rcs_m2 = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(CalibrateSingle(sample, &calibration));
  EXPECT_FALSE(CalibrateMultiple({}, &calibration));

  double value = 0.0;
  EXPECT_FALSE(InvertRcs(1.0, 10.0, calibration, &value));
  EXPECT_FALSE(EvaluateRadiometricErrorDb(0.0, 1.0, &value));
}

}  // namespace
}  // namespace calibration
}  // namespace sar
