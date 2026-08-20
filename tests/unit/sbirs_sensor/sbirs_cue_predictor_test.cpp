#include <gtest/gtest.h>

#include <limits>

#include "sbirs_sensor/pipeline/SbirsCuePredictor.h"

namespace sbirs_sensor {
namespace pipeline {
namespace {

TEST(SbirsCuePredictorTest, FirstSampleAndZeroLatencyFallBackToCurrentMeasurement) {
  SbirsCuePredictor predictor;
  const SbirsCuePrediction first = predictor.Update(1U, 10.0f, 2.0f, 1.0f, 0.5f);
  EXPECT_FLOAT_EQ(first.command_azimuth_deg, 10.0f);
  EXPECT_FLOAT_EQ(first.command_elevation_deg, 2.0f);
  EXPECT_FALSE(first.used_motion_prediction);

  const SbirsCuePrediction zero_latency = predictor.Update(1U, 12.0f, 3.0f, 1.0f, 0.0f);
  EXPECT_FLOAT_EQ(zero_latency.command_azimuth_deg, 12.0f);
  EXPECT_FLOAT_EQ(zero_latency.command_elevation_deg, 3.0f);
  EXPECT_FALSE(zero_latency.used_motion_prediction);
}

TEST(SbirsCuePredictorTest, ConstantAngularVelocityPredictsLatencyAhead) {
  SbirsCuePredictor predictor;
  predictor.Update(7U, 10.0f, 4.0f, 1.0f, 0.5f);
  const SbirsCuePrediction prediction = predictor.Update(7U, 12.0f, 5.0f, 1.0f, 0.5f);
  EXPECT_FLOAT_EQ(prediction.angular_rate_azimuth_deg_per_sec, 2.0f);
  EXPECT_FLOAT_EQ(prediction.angular_rate_elevation_deg_per_sec, 1.0f);
  EXPECT_FLOAT_EQ(prediction.command_azimuth_deg, 13.0f);
  EXPECT_FLOAT_EQ(prediction.command_elevation_deg, 5.5f);
  EXPECT_TRUE(prediction.used_motion_prediction);
}

TEST(SbirsCuePredictorTest, AzimuthUsesShortestPathAcrossWrap) {
  SbirsCuePredictor predictor;
  predictor.Update(3U, 179.0f, 0.0f, 1.0f, 1.0f);
  const SbirsCuePrediction prediction = predictor.Update(3U, -179.0f, 0.0f, 1.0f, 1.0f);
  EXPECT_FLOAT_EQ(prediction.angular_rate_azimuth_deg_per_sec, 2.0f);
  EXPECT_FLOAT_EQ(prediction.command_azimuth_deg, -177.0f);
}

TEST(SbirsCuePredictorTest, InvalidDtFallsBackAndStillRefreshesHistory) {
  SbirsCuePredictor predictor;
  predictor.Update(4U, 1.0f, 1.0f, 1.0f, 1.0f);
  const SbirsCuePrediction invalid =
      predictor.Update(4U, 5.0f, 2.0f, std::numeric_limits<float>::quiet_NaN(), 1.0f);
  EXPECT_FALSE(invalid.used_motion_prediction);
  EXPECT_FLOAT_EQ(invalid.command_azimuth_deg, 5.0f);

  const SbirsCuePrediction recovered = predictor.Update(4U, 7.0f, 3.0f, 1.0f, 1.0f);
  EXPECT_FLOAT_EQ(recovered.command_azimuth_deg, 9.0f);
  EXPECT_FLOAT_EQ(recovered.command_elevation_deg, 4.0f);
}

TEST(SbirsCuePredictorTest, TargetStateIsIsolatedAndReleaseDropsOnlyOneTarget) {
  SbirsCuePredictor predictor;
  predictor.Update(1U, 0.0f, 0.0f, 1.0f, 1.0f);
  predictor.Update(2U, 100.0f, 10.0f, 1.0f, 1.0f);
  predictor.Release(1U);

  EXPECT_FALSE(predictor.Update(1U, 2.0f, 0.0f, 1.0f, 1.0f).used_motion_prediction);
  const SbirsCuePrediction second = predictor.Update(2U, 101.0f, 12.0f, 1.0f, 1.0f);
  EXPECT_FLOAT_EQ(second.command_azimuth_deg, 102.0f);
  EXPECT_FLOAT_EQ(second.command_elevation_deg, 14.0f);
}

TEST(SbirsCuePredictorTest, CaptureRestorePreservesPerTargetHistory) {
  SbirsCuePredictor source;
  source.Update(9U, 30.0f, -2.0f, 1.0f, 0.25f);
  const SbirsCuePredictorSnapshot snapshot = source.Capture();

  SbirsCuePredictor restored;
  restored.Restore(snapshot);
  const SbirsCuePrediction prediction = restored.Update(9U, 34.0f, 0.0f, 2.0f, 0.5f);
  EXPECT_FLOAT_EQ(prediction.command_azimuth_deg, 35.0f);
  EXPECT_FLOAT_EQ(prediction.command_elevation_deg, 0.5f);
}

TEST(SbirsCuePredictorTest, ClearDropsAllTargetHistory) {
  SbirsCuePredictor predictor;
  predictor.Update(1U, 0.0f, 0.0f, 1.0f, 1.0f);
  predictor.Update(2U, 1.0f, 1.0f, 1.0f, 1.0f);
  predictor.Clear();
  EXPECT_TRUE(predictor.Capture().targets.empty());
  EXPECT_FALSE(predictor.Update(2U, 2.0f, 2.0f, 1.0f, 1.0f).used_motion_prediction);
}

}  // namespace
}  // namespace pipeline
}  // namespace sbirs_sensor
