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

TEST(SarRadiometricCalibrationTest, ExplicitObservationValidatesSourceAndConvertsAtomically) {
  signal::ComplexMatrix image;
  image.rows = 2U;
  image.cols = 2U;
  image.values = {signal::ComplexSample(1.0, 0.0), signal::ComplexSample(0.0, 2.0),
                  signal::ComplexSample(0.0, 0.0), signal::ComplexSample(3.0, 4.0)};
  CalibrationObservationRequest request;
  request.observation_id = "cal-1";
  request.known_rcs_m2 = 4.0;
  request.slant_range_m = 20.0;
  request.image_row = 1U;
  request.image_col = 1U;
  request.aperture_start_pulse_id = 10U;
  request.aperture_end_pulse_id = 18U;
  request.weight = 2.0;
  CalibrationObservation observation;
  ASSERT_TRUE(BuildCalibrationObservation(request, image, &observation));
  EXPECT_EQ(observation.observation_id, "cal-1");
  EXPECT_DOUBLE_EQ(observation.image_power, 25.0);

  CalibrationObservation same_math = observation;
  same_math.observation_id = "cal-2";
  std::vector<CalibrationSample> samples;
  ASSERT_TRUE(ConvertObservationsToSamples({observation, same_math}, &samples));
  ASSERT_EQ(samples.size(), 2U);
  EXPECT_EQ(samples[0].image_power, samples[1].image_power);

  request.image_is_normalized = true;
  EXPECT_FALSE(BuildCalibrationObservation(request, image, &observation));
  request.image_is_normalized = false;
  request.observation_id.clear();
  EXPECT_FALSE(BuildCalibrationObservation(request, image, &observation));
  request.observation_id = "cal-1";
  request.image_row = 2U;
  EXPECT_FALSE(BuildCalibrationObservation(request, image, &observation));

  same_math.observation_id.clear();
  EXPECT_FALSE(ConvertObservationsToSamples({observation, same_math}, &samples));
  EXPECT_EQ(samples.size(), 2U);
}

TEST(SarRadiometricCalibrationTest, InternalExecutorMatchesPathAndFailsAtomically) {
  signal::ComplexMatrix image;
  image.rows = 1U;
  image.cols = 2U;
  image.values = {signal::ComplexSample(2.0, 0.0), signal::ComplexSample(0.0, 3.0)};
  CalibrationExecutionRequest first;
  first.request_id = "request-1";
  first.image_path = CalibrationImagePath::kGbp;
  first.observation.observation_id = "observation-1";
  first.observation.known_rcs_m2 = 4.0;
  first.observation.slant_range_m = 10.0;
  first.observation.image_col = 0U;
  CalibrationExecutionRequest second = first;
  second.request_id = "request-2";
  second.observation.observation_id = "observation-2";
  second.observation.known_rcs_m2 = 9.0;
  second.observation.image_col = 1U;
  second.observation.weight = 2.0;

  CalibrationExecutionResult result;
  ASSERT_TRUE(ExecuteCalibrationRequests(CalibrationImagePath::kGbp, image, {first, second},
                                         &result));
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(result.failure, CalibrationExecutionFailure::kNone);
  EXPECT_EQ(result.request_count, 2U);
  ASSERT_EQ(result.residuals.size(), 2U);
  EXPECT_EQ(result.residuals[0].request_id, "request-1");
  EXPECT_EQ(result.residuals[1].observation_id, "observation-2");

  const CalibrationExecutionResult successful = result;
  EXPECT_FALSE(
      ExecuteCalibrationRequests(CalibrationImagePath::kRda, image, {first, second}, &result));
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.failure, CalibrationExecutionFailure::kImagePathMismatch);
  EXPECT_TRUE(result.residuals.empty());
  EXPECT_TRUE(successful.valid);

  second.observation.image_col = 2U;
  EXPECT_FALSE(
      ExecuteCalibrationRequests(CalibrationImagePath::kGbp, image, {first, second}, &result));
  EXPECT_EQ(result.failure, CalibrationExecutionFailure::kObservationBuildFailed);
  EXPECT_TRUE(result.residuals.empty());

  EXPECT_FALSE(ExecuteCalibrationRequests(CalibrationImagePath::kGbp, image, {}, &result));
  EXPECT_EQ(result.failure, CalibrationExecutionFailure::kEmptyRequestList);
}

}  // namespace
}  // namespace calibration
}  // namespace sar
