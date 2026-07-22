#include <gtest/gtest.h>

#include <cmath>

#include "airborne_radar/signal/detection/ArDetectionCellResolver.h"

namespace airborne_radar {
namespace signal {
namespace detection {
namespace {

ArDetectionCellConfig MakeConfig() {
  ArDetectionCellConfig config;
  config.carrier_frequency_hz = 10.0e9;
  config.matched_filter_bandwidth_hz = 10.0e6;
  config.pulse_width_s = 10.0e-6;
  config.radiated_peak_power_w = 1.0e6;
  config.one_way_antenna_gain_dbi = 35.0;
  config.receiver_loss_db = 2.0;
  config.receiver_noise_figure_db = 4.0;
  return config;
}

ArDetectionCellTarget MakeTarget(double range_m) {
  ArDetectionCellTarget target;
  target.range_m = range_m;
  target.closing_radial_velocity_mps = 300.0;
  target.rcs_m2 = 1.0;
  target.effective_pulse_count = 8U;
  return target;
}

TEST(ArDetectionCellResolverTest, EchoRangeDelayDopplerAndCompressionArePhysical) {
  const oneq::electromagnetics::RfEmissionIdentity own{1U, 2U, 3U};
  ArDetectionCellResult near_cell;
  ArDetectionCellResult far_cell;
  ASSERT_TRUE(
      TryResolveArDetectionCell(MakeConfig(), MakeTarget(10000.0), own, {}, 0.0, &near_cell));
  ASSERT_TRUE(
      TryResolveArDetectionCell(MakeConfig(), MakeTarget(20000.0), own, {}, 0.0, &far_cell));
  EXPECT_NEAR(10.0 * std::log10(far_cell.echo_power_w / near_cell.echo_power_w), -12.0412, 1.0e-3);
  EXPECT_NEAR(far_cell.echo_delay_s / near_cell.echo_delay_s, 2.0, 1.0e-12);
  EXPECT_GT(near_cell.two_way_doppler_shift_hz, 0.0);
  EXPECT_DOUBLE_EQ(near_cell.pulse_compression_gain, 100.0);
}

TEST(ArDetectionCellResolverTest, SuppressionOnlyChangesSinrAndPulseCountDoesNotDoubleIntegrate) {
  const oneq::electromagnetics::RfEmissionIdentity own{1U, 2U, 3U};
  ArDetectionCellTarget target = MakeTarget(10000.0);
  ArDetectionCellResult baseline;
  ASSERT_TRUE(TryResolveArDetectionCell(MakeConfig(), target, own, {}, 0.0, &baseline));

  oneq::electromagnetics::RfIncidentLinkResult jammer;
  jammer.identity = oneq::electromagnetics::RfEmissionIdentity{4U, 5U, 6U};
  jammer.received_power_w = baseline.thermal_noise_power_w * 10.0;
  ArDetectionCellResult jammed;
  ASSERT_TRUE(TryResolveArDetectionCell(MakeConfig(), target, own, {jammer}, 0.0, &jammed));
  EXPECT_LT(jammed.processed_single_pulse_sinr_db, baseline.processed_single_pulse_sinr_db);
  EXPECT_DOUBLE_EQ(jammed.echo_power_w, baseline.echo_power_w);

  target.effective_pulse_count = 64U;
  ArDetectionCellResult more_pulses;
  ASSERT_TRUE(TryResolveArDetectionCell(MakeConfig(), target, own, {jammer}, 0.0, &more_pulses));
  EXPECT_DOUBLE_EQ(more_pulses.processed_single_pulse_sinr_linear,
                   jammed.processed_single_pulse_sinr_linear);
  EXPECT_EQ(more_pulses.effective_pulse_count, 64U);
}

TEST(ArDetectionCellResolverTest, InvalidInputIsRejectedAtomically) {
  ArDetectionCellResult untouched;
  untouched.echo_power_w = 77.0;
  ArDetectionCellTarget target = MakeTarget(0.0);
  EXPECT_FALSE(TryResolveArDetectionCell(MakeConfig(), target,
                                         oneq::electromagnetics::RfEmissionIdentity{1U, 2U, 3U}, {},
                                         0.0, &untouched));
  EXPECT_DOUBLE_EQ(untouched.echo_power_w, 77.0);
}

}  // namespace
}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar
