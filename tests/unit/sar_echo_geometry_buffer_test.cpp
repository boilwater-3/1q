#include <gtest/gtest.h>

#include <cmath>
#include <limits>
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

TEST(SarGeometryTest, ZeroPerturbationL2ExactlyMatchesL1Track) {
  geometry::StraightStripmapTrackConfig ideal;
  ideal.start_position_m.x_m = -2.0;
  ideal.start_position_m.y_m = 1000.0;
  ideal.velocity_x_mps = 100.0;
  ideal.prf_hz = 20.0;
  ideal.first_pulse_id = 40U;
  ideal.pulse_count = 16U;
  std::vector<geometry::PlatformPulseState> l1;
  ASSERT_TRUE(geometry::GenerateStraightStripmapTrack(ideal, &l1));

  geometry::PerturbedStripmapTrackConfig config;
  config.ideal = ideal;
  config.random_seed = 1234U;
  std::vector<geometry::PlatformPulseState> l2;
  geometry::TrajectoryErrorDiagnostics diagnostics;
  ASSERT_TRUE(geometry::GeneratePerturbedStripmapTrack(config, &l2, &diagnostics));

  ASSERT_EQ(l2.size(), l1.size());
  for (std::size_t index = 0U; index < l1.size(); ++index) {
    EXPECT_DOUBLE_EQ(l2[index].position_m.x_m, l1[index].position_m.x_m);
    EXPECT_DOUBLE_EQ(l2[index].position_m.y_m, l1[index].position_m.y_m);
    EXPECT_DOUBLE_EQ(l2[index].position_m.z_m, l1[index].position_m.z_m);
    EXPECT_DOUBLE_EQ(l2[index].velocity_x_mps, l1[index].velocity_x_mps);
    EXPECT_DOUBLE_EQ(l2[index].velocity_y_mps, 0.0);
    EXPECT_DOUBLE_EQ(l2[index].velocity_z_mps, 0.0);
  }
  EXPECT_DOUBLE_EQ(diagnostics.max_position_error_m, 0.0);
  EXPECT_DOUBLE_EQ(diagnostics.rms_position_error_m, 0.0);
}

TEST(SarGeometryTest, PerturbedL2TrackIsDeterministicContinuousAndNonzero) {
  geometry::PerturbedStripmapTrackConfig config;
  config.ideal.start_position_m.y_m = 1000.0;
  config.ideal.velocity_x_mps = 100.0;
  config.ideal.prf_hz = 50.0;
  config.ideal.pulse_count = 64U;
  config.velocity_error_stddev_x_mps = 0.5;
  config.velocity_error_stddev_y_mps = 0.4;
  config.velocity_error_stddev_z_mps = 0.2;
  config.random_seed = 77U;

  std::vector<geometry::PlatformPulseState> first;
  std::vector<geometry::PlatformPulseState> second;
  geometry::TrajectoryErrorDiagnostics first_diagnostics;
  geometry::TrajectoryErrorDiagnostics second_diagnostics;
  ASSERT_TRUE(geometry::GeneratePerturbedStripmapTrack(config, &first, &first_diagnostics));
  ASSERT_TRUE(geometry::GeneratePerturbedStripmapTrack(config, &second, &second_diagnostics));
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t index = 0U; index < first.size(); ++index) {
    EXPECT_DOUBLE_EQ(first[index].position_m.x_m, second[index].position_m.x_m);
    EXPECT_DOUBLE_EQ(first[index].position_m.y_m, second[index].position_m.y_m);
    EXPECT_DOUBLE_EQ(first[index].position_m.z_m, second[index].position_m.z_m);
    if (index > 0U) {
      EXPECT_LT(geometry::Distance(first[index - 1U].position_m, first[index].position_m), 3.0);
    }
  }
  EXPECT_GT(first_diagnostics.max_position_error_m, 0.0);
  EXPECT_GT(first_diagnostics.rms_position_error_m, 0.0);
  EXPECT_GT(first_diagnostics.max_velocity_error_mps, 0.0);
  EXPECT_DOUBLE_EQ(first_diagnostics.max_position_error_m, second_diagnostics.max_position_error_m);

  config.random_seed = 78U;
  std::vector<geometry::PlatformPulseState> different;
  geometry::TrajectoryErrorDiagnostics different_diagnostics;
  ASSERT_TRUE(geometry::GeneratePerturbedStripmapTrack(config, &different, &different_diagnostics));
  EXPECT_NE(first.back().position_m.y_m, different.back().position_m.y_m);
}

TEST(SarGeometryTest, StraightWaypointTrackExactlyMatchesL1Track) {
  geometry::StraightStripmapTrackConfig l1_config;
  l1_config.start_position_m.x_m = -50.0;
  l1_config.start_position_m.y_m = 1000.0;
  l1_config.velocity_x_mps = 100.0;
  l1_config.prf_hz = 4.0;
  l1_config.first_pulse_id = 20U;
  l1_config.pulse_count = 5U;
  std::vector<geometry::PlatformPulseState> l1;
  ASSERT_TRUE(geometry::GenerateStraightStripmapTrack(l1_config, &l1));

  geometry::WaypointTrackConfig l3_config;
  geometry::Waypoint start;
  start.time_s = 0.0;
  start.position_m = l1_config.start_position_m;
  geometry::Waypoint end;
  end.time_s = 1.0;
  end.position_m = l1_config.start_position_m;
  end.position_m.x_m += 100.0;
  l3_config.waypoints.push_back(start);
  l3_config.waypoints.push_back(end);
  l3_config.first_pulse_id = l1_config.first_pulse_id;
  for (std::size_t index = 0U; index < l1_config.pulse_count; ++index) {
    l3_config.pulse_times_s.push_back(static_cast<double>(index) / l1_config.prf_hz);
  }

  std::vector<geometry::PlatformPulseState> l3;
  ASSERT_TRUE(geometry::GenerateWaypointTrack(l3_config, &l3));
  ASSERT_EQ(l3.size(), l1.size());
  for (std::size_t index = 0U; index < l1.size(); ++index) {
    EXPECT_EQ(l3[index].pulse_id, l1[index].pulse_id);
    EXPECT_DOUBLE_EQ(l3[index].time_s, l1[index].time_s);
    EXPECT_DOUBLE_EQ(l3[index].position_m.x_m, l1[index].position_m.x_m);
    EXPECT_DOUBLE_EQ(l3[index].position_m.y_m, l1[index].position_m.y_m);
    EXPECT_DOUBLE_EQ(l3[index].position_m.z_m, l1[index].position_m.z_m);
    EXPECT_DOUBLE_EQ(l3[index].velocity_x_mps, l1[index].velocity_x_mps);
    EXPECT_DOUBLE_EQ(l3[index].velocity_y_mps, l1[index].velocity_y_mps);
    EXPECT_DOUBLE_EQ(l3[index].velocity_z_mps, l1[index].velocity_z_mps);
  }
}

TEST(SarGeometryTest, WaypointTrackPreservesTurnAndNonuniformPulseTimes) {
  geometry::WaypointTrackConfig config;
  geometry::Waypoint first;
  first.time_s = 0.0;
  geometry::Waypoint turn;
  turn.time_s = 1.0;
  turn.position_m.x_m = 100.0;
  geometry::Waypoint last;
  last.time_s = 3.0;
  last.position_m.x_m = 100.0;
  last.position_m.y_m = 100.0;
  config.waypoints.push_back(first);
  config.waypoints.push_back(turn);
  config.waypoints.push_back(last);
  config.pulse_times_s = {0.0, 0.25, 1.0, 1.5, 3.0};
  config.first_pulse_id = 100U;

  std::vector<geometry::PlatformPulseState> pulses;
  ASSERT_TRUE(geometry::GenerateWaypointTrack(config, &pulses));
  ASSERT_EQ(pulses.size(), config.pulse_times_s.size());
  std::vector<geometry::PlatformPulseState> repeated;
  ASSERT_TRUE(geometry::GenerateWaypointTrack(config, &repeated));
  ASSERT_EQ(repeated.size(), pulses.size());
  EXPECT_DOUBLE_EQ(pulses[1].position_m.x_m, 25.0);
  EXPECT_DOUBLE_EQ(pulses[1].position_m.y_m, 0.0);
  EXPECT_DOUBLE_EQ(pulses[1].velocity_x_mps, 100.0);
  EXPECT_DOUBLE_EQ(pulses[1].velocity_y_mps, 0.0);
  EXPECT_DOUBLE_EQ(pulses[2].position_m.x_m, 100.0);
  EXPECT_DOUBLE_EQ(pulses[2].position_m.y_m, 0.0);
  EXPECT_DOUBLE_EQ(pulses[2].velocity_x_mps, 0.0);
  EXPECT_DOUBLE_EQ(pulses[2].velocity_y_mps, 50.0);
  EXPECT_DOUBLE_EQ(pulses[3].position_m.x_m, 100.0);
  EXPECT_DOUBLE_EQ(pulses[3].position_m.y_m, 25.0);
  EXPECT_DOUBLE_EQ(pulses.back().position_m.y_m, 100.0);
  for (std::size_t index = 0U; index < pulses.size(); ++index) {
    EXPECT_EQ(pulses[index].pulse_id, config.first_pulse_id + index);
    EXPECT_DOUBLE_EQ(pulses[index].time_s, config.pulse_times_s[index]);
    EXPECT_DOUBLE_EQ(repeated[index].position_m.x_m, pulses[index].position_m.x_m);
    EXPECT_DOUBLE_EQ(repeated[index].position_m.y_m, pulses[index].position_m.y_m);
    EXPECT_DOUBLE_EQ(repeated[index].position_m.z_m, pulses[index].position_m.z_m);
    EXPECT_DOUBLE_EQ(repeated[index].velocity_x_mps, pulses[index].velocity_x_mps);
    EXPECT_DOUBLE_EQ(repeated[index].velocity_y_mps, pulses[index].velocity_y_mps);
    EXPECT_DOUBLE_EQ(repeated[index].velocity_z_mps, pulses[index].velocity_z_mps);
  }
}

TEST(SarGeometryTest, WaypointTrackRejectsInvalidTimeContracts) {
  geometry::WaypointTrackConfig config;
  geometry::Waypoint first;
  first.time_s = 0.0;
  geometry::Waypoint last;
  last.time_s = 1.0;
  last.position_m.x_m = 1.0;
  config.waypoints = {first, last};
  config.pulse_times_s = {0.0, 0.5, 1.0};
  std::vector<geometry::PlatformPulseState> pulses;
  ASSERT_TRUE(geometry::GenerateWaypointTrack(config, &pulses));

  config.waypoints.pop_back();
  EXPECT_FALSE(geometry::GenerateWaypointTrack(config, &pulses));
  config.waypoints.push_back(last);
  config.waypoints.back().time_s = 0.0;
  EXPECT_FALSE(geometry::GenerateWaypointTrack(config, &pulses));
  config.waypoints.back().time_s = 1.0;
  config.pulse_times_s = {0.0, 0.5, 0.5};
  EXPECT_FALSE(geometry::GenerateWaypointTrack(config, &pulses));
  config.pulse_times_s = {-0.1, 0.5, 1.0};
  EXPECT_FALSE(geometry::GenerateWaypointTrack(config, &pulses));
  config.pulse_times_s = {0.0, 0.5, 1.1};
  EXPECT_FALSE(geometry::GenerateWaypointTrack(config, &pulses));
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

TEST(SarEchoTest, GeneratePointTargetRawEchoRejectsNanSampleRate) {
  echo::RawEchoConfig config;
  config.sample_rate_hz = std::numeric_limits<double>::quiet_NaN();
  config.carrier_frequency_hz = 10.0e9;
  config.range_sample_count = 32U;

  echo::RawEchoResult result;
  EXPECT_FALSE(echo::GeneratePointTargetRawEcho(
      config, MakePlatformAtOrigin(),
      {MakeTargetAtDelay(10U, 100.0e6)},
      {signal::ComplexSample(1.0, 0.0)}, &result));
}

TEST(SarEchoTest, GeneratePointTargetRawEchoRejectsInfCarrierFrequency) {
  echo::RawEchoConfig config;
  config.sample_rate_hz = 100.0e6;
  config.carrier_frequency_hz = std::numeric_limits<double>::infinity();
  config.range_sample_count = 32U;

  echo::RawEchoResult result;
  EXPECT_FALSE(echo::GeneratePointTargetRawEcho(
      config, MakePlatformAtOrigin(),
      {MakeTargetAtDelay(10U, 100.0e6)},
      {signal::ComplexSample(1.0, 0.0)}, &result));
}

TEST(SarEchoTest, GeneratePointTargetRawEchoRejectsNanCarrierFrequency) {
  echo::RawEchoConfig config;
  config.sample_rate_hz = 100.0e6;
  config.carrier_frequency_hz = std::numeric_limits<double>::quiet_NaN();
  config.range_sample_count = 32U;

  echo::RawEchoResult result;
  EXPECT_FALSE(echo::GeneratePointTargetRawEcho(
      config, MakePlatformAtOrigin(),
      {MakeTargetAtDelay(10U, 100.0e6)},
      {signal::ComplexSample(1.0, 0.0)}, &result));
}

TEST(SarEchoTest, GeneratePointTargetRawEchoRejectsZeroCarrierFrequency) {
  echo::RawEchoConfig config;
  config.sample_rate_hz = 100.0e6;
  config.carrier_frequency_hz = 0.0;
  config.range_sample_count = 32U;

  echo::RawEchoResult result;
  EXPECT_FALSE(echo::GeneratePointTargetRawEcho(
      config, MakePlatformAtOrigin(),
      {MakeTargetAtDelay(10U, 100.0e6)},
      {signal::ComplexSample(1.0, 0.0)}, &result));
}

TEST(SarGeometryTest, GenerateStraightStripmapTrackRejectsNanPrf) {
  geometry::StraightStripmapTrackConfig config;
  config.start_position_m = {10.0, 100.0, 2000.0};
  config.velocity_x_mps = 150.0;
  config.prf_hz = std::numeric_limits<double>::quiet_NaN();
  config.pulse_count = 4U;

  std::vector<geometry::PlatformPulseState> pulses;
  EXPECT_FALSE(geometry::GenerateStraightStripmapTrack(config, &pulses));
}

TEST(SarGeometryTest, GenerateStraightStripmapTrackRejectsInfPrf) {
  geometry::StraightStripmapTrackConfig config;
  config.start_position_m = {10.0, 100.0, 2000.0};
  config.velocity_x_mps = 150.0;
  config.prf_hz = std::numeric_limits<double>::infinity();
  config.pulse_count = 4U;

  std::vector<geometry::PlatformPulseState> pulses;
  EXPECT_FALSE(geometry::GenerateStraightStripmapTrack(config, &pulses));
}

TEST(SarGeometryTest, AdvanceFractionalPrfRejectsNanDt) {
  geometry::FractionalPrfState state;
  std::uint32_t emitted = 0U;
  EXPECT_FALSE(geometry::AdvanceFractionalPrf(
      std::numeric_limits<double>::quiet_NaN(), 12.5, &state, &emitted));
}

TEST(SarGeometryTest, AdvanceFractionalPrfRejectsNanPrf) {
  geometry::FractionalPrfState state;
  std::uint32_t emitted = 0U;
  EXPECT_FALSE(geometry::AdvanceFractionalPrf(
      0.1, std::numeric_limits<double>::quiet_NaN(), &state, &emitted));
}

}  // namespace
}  // namespace sar
