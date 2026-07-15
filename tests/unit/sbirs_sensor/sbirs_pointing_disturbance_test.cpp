#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "sbirs_sensor/pipeline/SbirsPointingDisturbance.h"

namespace sbirs_sensor {
namespace pipeline {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

double Rms(const std::vector<double>& values) {
  double sum = 0.0;
  for (double value : values) {
    sum += value * value;
  }
  return std::sqrt(sum / static_cast<double>(values.size()));
}

double LagOneCorrelation(const std::vector<double>& values) {
  double mean = 0.0;
  for (double value : values) {
    mean += value;
  }
  mean /= static_cast<double>(values.size());
  double numerator = 0.0;
  double denominator = 0.0;
  for (std::size_t i = 1; i < values.size(); ++i) {
    numerator += (values[i - 1] - mean) * (values[i] - mean);
  }
  for (double value : values) {
    denominator += (value - mean) * (value - mean);
  }
  return numerator / denominator;
}

SbirsPointingDisturbanceParameters CommonParameters(double sigma, double tau) {
  SbirsPointingDisturbanceParameters parameters;
  parameters.common_attitude_sigma_deg = sigma;
  parameters.common_attitude_correlation_time_s = tau;
  return parameters;
}

TEST(SbirsPointingDisturbanceTest, ZeroAmplitudeProducesExactZero) {
  SbirsPointingDisturbance disturbance(2, 7U);
  SbirsPointingDisturbanceParameters parameters;
  ASSERT_TRUE(disturbance.Advance(0.1, parameters));
  for (int channel_id = 0; channel_id < 2; ++channel_id) {
    SbirsPointingDisturbanceSample sample;
    ASSERT_TRUE(disturbance.Sample(channel_id, parameters, &sample));
    EXPECT_DOUBLE_EQ(sample.common.azimuth_deg, 0.0);
    EXPECT_DOUBLE_EQ(sample.common.elevation_deg, 0.0);
    EXPECT_DOUBLE_EQ(sample.channel.azimuth_deg, 0.0);
    EXPECT_DOUBLE_EQ(sample.channel.elevation_deg, 0.0);
  }
}

TEST(SbirsPointingDisturbanceTest, GaussMarkovMatchesStationaryRmsAndLagOne) {
  constexpr int kSampleCount = 8192;
  constexpr int kBurnIn = 1024;
  constexpr double kDt = 0.01;
  constexpr double kSigma = 0.2;
  constexpr double kTau = 0.5;
  SbirsPointingDisturbance disturbance(1, 11U);
  const SbirsPointingDisturbanceParameters parameters = CommonParameters(kSigma, kTau);
  std::vector<double> samples;
  samples.reserve(kSampleCount - kBurnIn);
  for (int i = 0; i < kSampleCount; ++i) {
    ASSERT_TRUE(disturbance.Advance(kDt, parameters));
    SbirsPointingDisturbanceSample sample;
    ASSERT_TRUE(disturbance.Sample(0, parameters, &sample));
    if (i >= kBurnIn) {
      samples.push_back(sample.common.azimuth_deg);
    }
  }
  EXPECT_NEAR(Rms(samples), kSigma, 0.1 * kSigma);
  EXPECT_NEAR(LagOneCorrelation(samples), std::exp(-kDt / kTau), 0.05);
}

TEST(SbirsPointingDisturbanceTest, WhiteNoiseBaselineHasNearZeroLagOne) {
  foundation::SbirsRandomSource random(11U);
  std::vector<double> samples;
  samples.reserve(8192);
  for (int i = 0; i < 8192; ++i) {
    samples.push_back(random.NextStandardNormal());
  }
  EXPECT_NEAR(LagOneCorrelation(samples), 0.0, 0.05);
}

TEST(SbirsPointingDisturbanceTest, DeterministicVibrationHasConfiguredAmplitudeAndFrequency) {
  constexpr int kSampleCount = 1000;
  constexpr double kDt = 0.01;
  constexpr double kFrequencyHz = 2.0;
  constexpr double kAmplitudeDeg = 0.3;
  SbirsPointingDisturbance disturbance(1, 17U);
  SbirsPointingDisturbanceParameters parameters;
  parameters.channel_vibration_amplitude_deg = kAmplitudeDeg;
  parameters.channel_vibration_frequency_hz = kFrequencyHz;
  double sine_projection = 0.0;
  double cosine_projection = 0.0;
  for (int i = 0; i < kSampleCount; ++i) {
    ASSERT_TRUE(disturbance.Advance(kDt, parameters));
    SbirsPointingDisturbanceSample sample;
    ASSERT_TRUE(disturbance.Sample(0, parameters, &sample));
    const double time_s = (i + 1) * kDt;
    sine_projection += sample.channel.azimuth_deg * std::sin(2.0 * kPi * kFrequencyHz * time_s);
    cosine_projection += sample.channel.azimuth_deg * std::cos(2.0 * kPi * kFrequencyHz * time_s);
  }
  const double recovered_amplitude =
      2.0 * std::hypot(sine_projection, cosine_projection) / kSampleCount;
  EXPECT_NEAR(recovered_amplitude, kAmplitudeDeg, 0.01 * kAmplitudeDeg);
}

TEST(SbirsPointingDisturbanceTest, ChannelsAreIndependentAndAdvanceWhileIdle) {
  SbirsPointingDisturbance disturbance(2, 23U);
  SbirsPointingDisturbanceParameters parameters;
  parameters.channel_pointing_sigma_deg = 0.1;
  parameters.channel_pointing_correlation_time_s = 0.5;
  ASSERT_TRUE(disturbance.Advance(0.1, parameters));
  SbirsPointingDisturbanceSample first;
  SbirsPointingDisturbanceSample second;
  ASSERT_TRUE(disturbance.Sample(0, parameters, &first));
  ASSERT_TRUE(disturbance.Sample(1, parameters, &second));
  EXPECT_DOUBLE_EQ(first.common.azimuth_deg, second.common.azimuth_deg);
  EXPECT_NE(first.channel.azimuth_deg, second.channel.azimuth_deg);
  const SbirsPointingDisturbanceSnapshot before = disturbance.Capture();
  ASSERT_TRUE(disturbance.Advance(0.1, parameters));
  const SbirsPointingDisturbanceSnapshot after = disturbance.Capture();
  EXPECT_GT(after.channels[0].elapsed_time_s, before.channels[0].elapsed_time_s);
  EXPECT_GT(after.channels[1].elapsed_time_s, before.channels[1].elapsed_time_s);
}

TEST(SbirsPointingDisturbanceTest, InvalidInputIsRejectedAtomically) {
  SbirsPointingDisturbance disturbance(1, 29U);
  SbirsPointingDisturbanceParameters parameters = CommonParameters(0.1, 1.0);
  ASSERT_TRUE(disturbance.Advance(0.1, parameters));
  const SbirsPointingDisturbanceSnapshot before = disturbance.Capture();
  parameters.common_attitude_sigma_deg = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(disturbance.Advance(0.1, parameters));
  const SbirsPointingDisturbanceSnapshot after = disturbance.Capture();
  EXPECT_DOUBLE_EQ(after.common.azimuth_deg, before.common.azimuth_deg);
  EXPECT_EQ(after.common.random_state, before.common.random_state);
  EXPECT_DOUBLE_EQ(after.channels[0].elapsed_time_s, before.channels[0].elapsed_time_s);
}

TEST(SbirsPointingDisturbanceTest, CaptureRestorePreservesExactContinuation) {
  SbirsPointingDisturbance uninterrupted(2, 31U);
  SbirsPointingDisturbanceParameters parameters = CommonParameters(0.1, 0.5);
  parameters.channel_pointing_sigma_deg = 0.2;
  parameters.channel_pointing_correlation_time_s = 0.7;
  parameters.channel_vibration_amplitude_deg = 0.05;
  parameters.channel_vibration_frequency_hz = 3.0;
  ASSERT_TRUE(uninterrupted.Advance(0.1, parameters));
  const SbirsPointingDisturbanceSnapshot snapshot = uninterrupted.Capture();
  SbirsPointingDisturbance restored(2, 1U);
  ASSERT_TRUE(restored.Restore(snapshot));
  ASSERT_TRUE(uninterrupted.Advance(0.2, parameters));
  ASSERT_TRUE(restored.Advance(0.2, parameters));
  for (int channel_id = 0; channel_id < 2; ++channel_id) {
    SbirsPointingDisturbanceSample expected;
    SbirsPointingDisturbanceSample actual;
    ASSERT_TRUE(uninterrupted.Sample(channel_id, parameters, &expected));
    ASSERT_TRUE(restored.Sample(channel_id, parameters, &actual));
    EXPECT_DOUBLE_EQ(actual.common.azimuth_deg, expected.common.azimuth_deg);
    EXPECT_DOUBLE_EQ(actual.common.elevation_deg, expected.common.elevation_deg);
    EXPECT_DOUBLE_EQ(actual.channel.azimuth_deg, expected.channel.azimuth_deg);
    EXPECT_DOUBLE_EQ(actual.channel.elevation_deg, expected.channel.elevation_deg);
  }
}

}  // namespace
}  // namespace pipeline
}  // namespace sbirs_sensor
