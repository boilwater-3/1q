// Copyright 2026. All Rights Reserved.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "1q/electromagnetics/RfLinkBudget.h"

namespace oneq {
namespace electromagnetics {
namespace {

RfEmission MakeEmission(double range_m, double power_w = 100.0) {
  RfEmission emission;
  emission.emission_id = 11;
  emission.entity_id = 101;
  emission.position_ecef_m.x_m = range_m;
  emission.antenna.boresight_ecef_unit.x = -1.0;
  emission.antenna.boresight_ecef_unit.y = 0.0;
  emission.antenna.boresight_ecef_unit.z = 0.0;
  emission.polarization = RfPolarization::kHorizontal;
  RfEmissionSegment segment;
  segment.duration_s = 1.0;
  segment.center_frequency_hz = 10.0e9;
  segment.bandwidth_hz = 10.0e6;
  segment.transmit_power_w = power_w;
  emission.segments.push_back(segment);
  return emission;
}

RfReceiverSite MakeReceiver() {
  RfReceiverSite receiver;
  receiver.entity_id = 202;
  receiver.antenna.boresight_ecef_unit.x = 1.0;
  receiver.antenna.boresight_ecef_unit.y = 0.0;
  receiver.antenna.boresight_ecef_unit.z = 0.0;
  receiver.polarization = RfPolarization::kHorizontal;
  receiver.window_duration_s = 1.0;
  receiver.center_frequency_hz = 10.0e9;
  receiver.bandwidth_hz = 10.0e6;
  return receiver;
}

double ToDb(double power_w) { return 10.0 * std::log10(power_w); }

TEST(RfLinkBudgetTest, DistanceDoublingLosesSixPointZeroTwoDb) {
  const RfReceiverSite receiver = MakeReceiver();
  RfLinkResult near_result;
  RfLinkResult far_result;
  ASSERT_TRUE(TryEvaluateRfLink(MakeEmission(1000.0), receiver, {}, &near_result));
  ASSERT_TRUE(TryEvaluateRfLink(MakeEmission(2000.0), receiver, {}, &far_result));

  EXPECT_NEAR(ToDb(near_result.total_received_power_w) - ToDb(far_result.total_received_power_w),
              6.0206, 1.0e-4);
}

TEST(RfLinkBudgetTest, TransmitPowerDoublingAddsThreePointZeroOneDb) {
  const RfReceiverSite receiver = MakeReceiver();
  RfLinkResult low_result;
  RfLinkResult high_result;
  ASSERT_TRUE(TryEvaluateRfLink(MakeEmission(1000.0, 100.0), receiver, {}, &low_result));
  ASSERT_TRUE(TryEvaluateRfLink(MakeEmission(1000.0, 200.0), receiver, {}, &high_result));

  EXPECT_NEAR(ToDb(high_result.total_received_power_w) - ToDb(low_result.total_received_power_w),
              3.0103, 1.0e-4);
}

TEST(RfLinkBudgetTest, TimeAndFrequencyOverlapScalePowerExactly) {
  RfEmission emission = MakeEmission(1000.0);
  emission.segments.front().start_time_s = 0.25;
  emission.segments.front().duration_s = 0.5;
  emission.segments.front().center_frequency_hz = 10.005e9;
  RfReceiverSite receiver = MakeReceiver();
  receiver.bandwidth_hz = 10.0e6;

  RfLinkResult result;
  ASSERT_TRUE(TryEvaluateRfLink(emission, receiver, {}, &result));
  ASSERT_EQ(result.segment_results.size(), 1U);
  EXPECT_DOUBLE_EQ(result.segment_results.front().frequency_overlap_fraction, 0.5);
  EXPECT_DOUBLE_EQ(result.segment_results.front().time_overlap_fraction, 0.5);
  EXPECT_NEAR(result.segment_results.front().received_power_w,
              0.25 * result.segment_results.front().received_power_before_overlap_w,
              result.segment_results.front().received_power_before_overlap_w * 1.0e-12);
}

TEST(RfLinkBudgetTest, DisjointTimeOrFrequencyContributesZeroPower) {
  RfReceiverSite receiver = MakeReceiver();
  RfEmission time_disjoint = MakeEmission(1000.0);
  time_disjoint.segments.front().start_time_s = 2.0;
  RfEmission frequency_disjoint = MakeEmission(1000.0);
  frequency_disjoint.emission_id = 12;
  frequency_disjoint.segments.front().center_frequency_hz = 11.0e9;

  RfLinkResult time_result;
  RfLinkResult frequency_result;
  ASSERT_TRUE(TryEvaluateRfLink(time_disjoint, receiver, {}, &time_result));
  ASSERT_TRUE(TryEvaluateRfLink(frequency_disjoint, receiver, {}, &frequency_result));
  EXPECT_DOUBLE_EQ(time_result.total_received_power_w, 0.0);
  EXPECT_DOUBLE_EQ(frequency_result.total_received_power_w, 0.0);
}

TEST(RfLinkBudgetTest, AggregationIsOrderIndependentAndRejectsDuplicateIdsAtomically) {
  RfLinkResult first;
  first.emission_id = 2;
  first.receiver_entity_id = 8;
  first.total_received_power_w = 1.0e-8;
  RfLinkResult second = first;
  second.emission_id = 1;
  second.total_received_power_w = 2.0e-8;
  std::vector<RfLinkResult> forward{first, second};
  std::vector<RfLinkResult> reverse{second, first};
  double forward_sum = -1.0;
  double reverse_sum = -1.0;
  ASSERT_TRUE(TryAggregateRfReceivedPower(forward, &forward_sum));
  ASSERT_TRUE(TryAggregateRfReceivedPower(reverse, &reverse_sum));
  EXPECT_DOUBLE_EQ(forward_sum, reverse_sum);
  EXPECT_DOUBLE_EQ(forward_sum, 3.0e-8);

  second.emission_id = first.emission_id;
  double unchanged = 17.0;
  EXPECT_FALSE(TryAggregateRfReceivedPower({first, second}, &unchanged));
  EXPECT_DOUBLE_EQ(unchanged, 17.0);
}

TEST(RfLinkBudgetTest, AntennaPolarizationAndCoSiteIsolationAreMonotonic) {
  RfReceiverSite receiver = MakeReceiver();
  RfEmission main_lobe = MakeEmission(1000.0);
  RfEmission sidelobe = main_lobe;
  sidelobe.emission_id = 12;
  sidelobe.antenna.boresight_ecef_unit.x = 0.0;
  sidelobe.antenna.boresight_ecef_unit.y = 1.0;
  RfEmission cross_polarized = main_lobe;
  cross_polarized.emission_id = 13;
  cross_polarized.polarization = RfPolarization::kVertical;
  RfLinkResult main_result;
  RfLinkResult side_result;
  RfLinkResult cross_result;
  ASSERT_TRUE(TryEvaluateRfLink(main_lobe, receiver, {}, &main_result));
  ASSERT_TRUE(TryEvaluateRfLink(sidelobe, receiver, {}, &side_result));
  ASSERT_TRUE(TryEvaluateRfLink(cross_polarized, receiver, {}, &cross_result));
  EXPECT_GT(main_result.total_received_power_w, side_result.total_received_power_w);
  EXPECT_GT(main_result.total_received_power_w, cross_result.total_received_power_w);

  RfEmission co_site = MakeEmission(0.0);
  co_site.entity_id = receiver.entity_id;
  co_site.position_ecef_m = receiver.position_ecef_m;
  receiver.has_co_site_isolation = true;
  receiver.co_site_isolation_db = 40.0;
  RfLinkResult low_isolation;
  ASSERT_TRUE(TryEvaluateRfLink(co_site, receiver, {}, &low_isolation));
  receiver.co_site_isolation_db = 80.0;
  RfLinkResult high_isolation;
  ASSERT_TRUE(TryEvaluateRfLink(co_site, receiver, {}, &high_isolation));
  EXPECT_GT(low_isolation.total_received_power_w, high_isolation.total_received_power_w);
  EXPECT_DOUBLE_EQ(low_isolation.segment_results.front().free_space_loss_db, 0.0);
}

TEST(RfLinkBudgetTest, FullPolarizationPairingAppliesMergeAndHalfRules) {
  const RfReceiverSite receiver = MakeReceiver();
  RfEmission full_tx = MakeEmission(1000.0);
  full_tx.polarization = RfPolarization::kFullPolarization;
  RfReceiverSite full_rx = MakeReceiver();
  full_rx.polarization = RfPolarization::kFullPolarization;
  RfLinkResult result;

  // 全极化发射 → 单极化接收：正交两分量只收其一（一半功率）。
  ASSERT_TRUE(TryEvaluateRfLink(full_tx, receiver, {}, &result));
  EXPECT_NEAR(result.polarization_mismatch_loss_db, 3.0103, 1.0e-4);

  // 固定极化发射 → 全极化接收：两正交通道合并收满。
  ASSERT_TRUE(TryEvaluateRfLink(MakeEmission(1000.0), full_rx, {}, &result));
  EXPECT_DOUBLE_EQ(result.polarization_mismatch_loss_db, 0.0);

  // 全极化对全极化：0 dB。
  ASSERT_TRUE(TryEvaluateRfLink(full_tx, full_rx, {}, &result));
  EXPECT_DOUBLE_EQ(result.polarization_mismatch_loss_db, 0.0);

  // 任一侧非极化公理不变：固定 3.01 dB。
  RfReceiverSite unpolarized_rx = MakeReceiver();
  unpolarized_rx.polarization = RfPolarization::kUnpolarized;
  ASSERT_TRUE(TryEvaluateRfLink(full_tx, unpolarized_rx, {}, &result));
  EXPECT_NEAR(result.polarization_mismatch_loss_db, 3.0103, 1.0e-4);
  RfEmission unpolarized_tx = MakeEmission(1000.0);
  unpolarized_tx.polarization = RfPolarization::kUnpolarized;
  ASSERT_TRUE(TryEvaluateRfLink(unpolarized_tx, full_rx, {}, &result));
  EXPECT_NEAR(result.polarization_mismatch_loss_db, 3.0103, 1.0e-4);

  // 探针转正：kFullPolarization 为合法枚举值，发射帧校验放行。
  EXPECT_TRUE(TryValidateRfEmissionFrame({full_tx}, 1.0));
}

TEST(RfLinkBudgetTest, InvalidInputsAndMissingCoSiteIsolationRejectAtomically) {
  RfReceiverSite receiver = MakeReceiver();
  RfEmission invalid = MakeEmission(1000.0);
  invalid.segments.front().transmit_power_w = -1.0;
  RfLinkResult unchanged;
  unchanged.emission_id = 999;
  EXPECT_FALSE(TryEvaluateRfLink(invalid, receiver, {}, &unchanged));
  EXPECT_EQ(unchanged.emission_id, 999U);

  invalid = MakeEmission(0.0);
  invalid.entity_id = receiver.entity_id;
  invalid.position_ecef_m = receiver.position_ecef_m;
  EXPECT_FALSE(TryEvaluateRfLink(invalid, receiver, {}, &unchanged));
  EXPECT_EQ(unchanged.emission_id, 999U);

  invalid = MakeEmission(1000.0);
  invalid.segments.front().center_frequency_hz = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(TryEvaluateRfLink(invalid, receiver, {}, &unchanged));
  EXPECT_EQ(unchanged.emission_id, 999U);
}

TEST(RfLinkBudgetTest, EmissionFrameValidationRejectsDuplicateAndOutOfCycleFacts) {
  RfEmission first = MakeEmission(1000.0);
  RfEmission second = MakeEmission(2000.0);
  EXPECT_FALSE(TryValidateRfEmissionFrame({first, second}, 1.0));

  second.emission_id = 12;
  EXPECT_TRUE(TryValidateRfEmissionFrame({first, second}, 1.0));

  second.segments.front().start_time_s = 0.75;
  second.segments.front().duration_s = 0.5;
  EXPECT_FALSE(TryValidateRfEmissionFrame({first, second}, 1.0));

  second.segments.front().start_time_s = 0.0;
  second.segments.front().duration_s = 1.0;
  second.polarization = static_cast<RfPolarization>(255);
  EXPECT_FALSE(TryValidateRfEmissionFrame({first, second}, 1.0));

  EXPECT_FALSE(TryValidateRfEmissionFrame({first}, std::numeric_limits<double>::quiet_NaN()));
}

}  // namespace
}  // namespace electromagnetics
}  // namespace oneq
