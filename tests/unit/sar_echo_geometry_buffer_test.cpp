#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "sar/echo/SarEcho.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/runtime/PulseRingBuffer.h"

namespace sar {
namespace {

constexpr double kSpeedOfLightMps = 299792458.0;
constexpr double kPi = 3.141592653589793238462643383279502884;

void ExpectNear(const signal::ComplexSample& actual, const signal::ComplexSample& expected,
                double tolerance) {
  EXPECT_NEAR(actual.real(), expected.real(), tolerance);
  EXPECT_NEAR(actual.imag(), expected.imag(), tolerance);
}

geometry::PlatformPulseState MakePlatformAtOrigin() {
  geometry::PlatformPulseState platform;
  platform.pulse_id = 10U;
  platform.position_m = {0.0, 0.0, 0.0};
  return platform;
}

echo::PointTarget MakeTargetAtDelay(std::size_t delay_sample, double sample_rate_hz,
                                    double rcs_m2 = 1.0) {
  const double range_m =
      static_cast<double>(delay_sample) * kSpeedOfLightMps / (2.0 * sample_rate_hz);
  echo::PointTarget target;
  target.position_m = {0.0, range_m, 0.0};
  target.rcs_m2 = rcs_m2;
  return target;
}

TEST(SarGeometryTest, StraightStripmapTrackUsesMonotonicPulseIdsAndConstantPrf) {
  geometry::StraightStripmapTrackConfig config;
  config.start_position_m = {10.0, 100.0, 2000.0};
  config.velocity_x_mps = 150.0;
  config.prf_hz = 50.0;
  config.first_pulse_id = 100U;
  config.pulse_count = 4U;

  std::vector<geometry::PlatformPulseState> pulses;
  ASSERT_TRUE(geometry::GenerateStraightStripmapTrack(config, &pulses));

  ASSERT_EQ(pulses.size(), 4U);
  for (std::size_t i = 0U; i < pulses.size(); ++i) {
    EXPECT_EQ(pulses[i].pulse_id, 100U + i);
    EXPECT_DOUBLE_EQ(pulses[i].time_s, static_cast<double>(i) / config.prf_hz);
    EXPECT_DOUBLE_EQ(pulses[i].position_m.x_m,
                     config.start_position_m.x_m + config.velocity_x_mps * pulses[i].time_s);
    EXPECT_DOUBLE_EQ(pulses[i].position_m.y_m, config.start_position_m.y_m);
    EXPECT_DOUBLE_EQ(pulses[i].position_m.z_m, config.start_position_m.z_m);
  }
}

TEST(SarGeometryTest, FractionalPrfCarryPreservesAveragePulseRate) {
  geometry::FractionalPrfState state;
  std::uint32_t emitted = 0U;

  ASSERT_TRUE(geometry::AdvanceFractionalPrf(0.1, 12.5, &state, &emitted));
  EXPECT_EQ(emitted, 1U);
  EXPECT_DOUBLE_EQ(state.carry_pulses, 0.25);

  ASSERT_TRUE(geometry::AdvanceFractionalPrf(0.1, 12.5, &state, &emitted));
  EXPECT_EQ(emitted, 1U);
  EXPECT_DOUBLE_EQ(state.carry_pulses, 0.5);

  ASSERT_TRUE(geometry::AdvanceFractionalPrf(0.4, 12.5, &state, &emitted));
  EXPECT_EQ(emitted, 5U);
  EXPECT_DOUBLE_EQ(state.carry_pulses, 0.5);
}

TEST(SarEchoTest, SinglePointTargetDelayPhaseAndAmplitudeAreCorrect) {
  const double sample_rate_hz = 100.0e6;
  const double carrier_hz = 10.0e9;
  const std::size_t delay_sample = 10U;
  const echo::PointTarget target = MakeTargetAtDelay(delay_sample, sample_rate_hz, 4.0);

  echo::RawEchoConfig config;
  config.sample_rate_hz = sample_rate_hz;
  config.carrier_frequency_hz = carrier_hz;
  config.range_sample_count = 32U;

  echo::RawEchoResult result;
  ASSERT_TRUE(echo::GeneratePointTargetRawEcho(config, MakePlatformAtOrigin(), {target},
                                               {signal::ComplexSample(1.0, 0.0)}, &result));

  ASSERT_EQ(result.samples.size(), config.range_sample_count);
  ASSERT_EQ(result.diagnostics.size(), 1U);
  EXPECT_EQ(result.diagnostics[0].delay_sample_index, delay_sample);
  EXPECT_FALSE(result.has_clipping);

  const double range_m = target.position_m.y_m;
  const double wavelength_m = kSpeedOfLightMps / carrier_hz;
  const double amplitude = std::sqrt(target.rcs_m2) / (range_m * range_m);
  const double phase = -4.0 * kPi * range_m / wavelength_m;
  ExpectNear(result.samples[delay_sample],
             signal::ComplexSample(amplitude * std::cos(phase), amplitude * std::sin(phase)),
             1.0e-12);
}

TEST(SarEchoTest, ThreeSeparatedTargetsProduceSeparatedBins) {
  const double sample_rate_hz = 100.0e6;
  echo::RawEchoConfig config;
  config.sample_rate_hz = sample_rate_hz;
  config.carrier_frequency_hz = 10.0e9;
  config.range_sample_count = 32U;

  echo::RawEchoResult result;
  ASSERT_TRUE(echo::GeneratePointTargetRawEcho(
      config, MakePlatformAtOrigin(),
      {MakeTargetAtDelay(5U, sample_rate_hz), MakeTargetAtDelay(10U, sample_rate_hz),
       MakeTargetAtDelay(15U, sample_rate_hz)},
      {signal::ComplexSample(1.0, 0.0)}, &result));

  ASSERT_EQ(result.diagnostics.size(), 3U);
  EXPECT_EQ(result.diagnostics[0].delay_sample_index, 5U);
  EXPECT_EQ(result.diagnostics[1].delay_sample_index, 10U);
  EXPECT_EQ(result.diagnostics[2].delay_sample_index, 15U);
  EXPECT_GT(std::abs(result.samples[5U]), 0.0);
  EXPECT_GT(std::abs(result.samples[10U]), 0.0);
  EXPECT_GT(std::abs(result.samples[15U]), 0.0);
  EXPECT_NE(std::abs(result.samples[5U]), std::abs(result.samples[10U]));
}

TEST(SarEchoTest, NearEdgeTargetReportsClippingDiagnostics) {
  const double sample_rate_hz = 100.0e6;
  echo::RawEchoConfig config;
  config.sample_rate_hz = sample_rate_hz;
  config.carrier_frequency_hz = 10.0e9;
  config.range_sample_count = 16U;

  echo::RawEchoResult result;
  ASSERT_TRUE(echo::GeneratePointTargetRawEcho(
      config, MakePlatformAtOrigin(), {MakeTargetAtDelay(14U, sample_rate_hz)},
      {signal::ComplexSample(1.0, 0.0), signal::ComplexSample(2.0, 0.0),
       signal::ComplexSample(3.0, 0.0), signal::ComplexSample(4.0, 0.0)},
      &result));

  ASSERT_EQ(result.diagnostics.size(), 1U);
  EXPECT_TRUE(result.has_clipping);
  EXPECT_TRUE(result.diagnostics[0].clipped);
  EXPECT_EQ(result.diagnostics[0].clipped_samples, 2U);
  EXPECT_GT(std::abs(result.samples[14U]), 0.0);
  EXPECT_GT(std::abs(result.samples[15U]), 0.0);
}

TEST(PulseRingBufferTest, RejectsDuplicateAndRequiresContiguousRangeReads) {
  runtime::PulseRingBuffer buffer(4U);
  EXPECT_TRUE(buffer.Push({10U, {signal::ComplexSample(10.0, 0.0)}}));
  EXPECT_TRUE(buffer.Push({11U, {signal::ComplexSample(11.0, 0.0)}}));
  EXPECT_FALSE(buffer.Push({11U, {signal::ComplexSample(11.0, 0.0)}}));
  EXPECT_TRUE(buffer.Push({13U, {signal::ComplexSample(13.0, 0.0)}}));

  std::vector<runtime::PulseRecord> output;
  EXPECT_TRUE(buffer.ReadRange(10U, 2U, &output));
  ASSERT_EQ(output.size(), 2U);
  EXPECT_EQ(output[0].pulse_id, 10U);
  EXPECT_EQ(output[1].pulse_id, 11U);

  EXPECT_FALSE(buffer.ReadRange(10U, 3U, &output));
  EXPECT_FALSE(buffer.ReadLatest(3U, &output));
}

TEST(PulseRingBufferTest, LatestReadsRequireContinuityAndOverflowStickyPersists) {
  runtime::PulseRingBuffer buffer(3U);
  EXPECT_TRUE(buffer.Push({1U, {}}));
  EXPECT_TRUE(buffer.Push({2U, {}}));
  EXPECT_TRUE(buffer.Push({3U, {}}));
  EXPECT_FALSE(buffer.overflow_sticky());

  EXPECT_TRUE(buffer.Push({4U, {}}));
  EXPECT_TRUE(buffer.overflow_sticky());
  EXPECT_EQ(buffer.size(), 3U);

  std::vector<runtime::PulseRecord> latest;
  ASSERT_TRUE(buffer.ReadLatest(3U, &latest));
  ASSERT_EQ(latest.size(), 3U);
  EXPECT_EQ(latest[0].pulse_id, 2U);
  EXPECT_EQ(latest[1].pulse_id, 3U);
  EXPECT_EQ(latest[2].pulse_id, 4U);
}

TEST(PulseRingBufferTest, RejectsOutOfOrderPulseIds) {
  runtime::PulseRingBuffer buffer(3U);
  EXPECT_TRUE(buffer.Push({5U, {}}));
  EXPECT_FALSE(buffer.Push({4U, {}}));
  EXPECT_FALSE(buffer.Push({5U, {}}));
}

}  // namespace
}  // namespace sar
