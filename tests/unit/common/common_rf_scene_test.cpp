#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "1q/electromagnetics/RfScene.h"

namespace oneq {
namespace electromagnetics {
namespace {

RfSceneEmission MakeNoiseEmission(std::uint64_t emission_id, double x_m, double power_w) {
  RfSceneEmission emission;
  emission.identity.platform_id = emission_id + 100U;
  emission.identity.equipment_id = emission_id + 200U;
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m.x_m = x_m;
  emission.antenna.boresight_ecef.x = -1.0;
  EXPECT_TRUE(TryCreateRfNoiseWaveform(10.0, 1.0, 10.0e9, 20.0e6, power_w, &emission.waveform));
  return emission;
}

RfSceneReceiverState MakeReceiver() {
  RfSceneReceiverState receiver;
  receiver.platform_id = 1U;
  receiver.equipment_id = 2U;
  receiver.window_start_time_s = 10.0;
  receiver.window_duration_s = 1.0;
  receiver.center_frequency_hz = 10.0e9;
  receiver.bandwidth_hz = 20.0e6;
  receiver.antenna.boresight_ecef.x = 1.0;
  return receiver;
}

TEST(RfSceneTest, DistanceAndPowerCharacterizationMatchOneWayBudget) {
  const RfSceneReceiverState receiver = MakeReceiver();
  RfIncidentLinkConfig config;
  RfIncidentLinkResult near_link;
  RfIncidentLinkResult far_link;
  RfIncidentLinkResult double_power_link;
  ASSERT_TRUE(
      TryEvaluateRfIncidentLink(MakeNoiseEmission(1U, 1000.0, 10.0), receiver, config, &near_link));
  ASSERT_TRUE(
      TryEvaluateRfIncidentLink(MakeNoiseEmission(2U, 2000.0, 10.0), receiver, config, &far_link));
  ASSERT_TRUE(TryEvaluateRfIncidentLink(MakeNoiseEmission(3U, 1000.0, 20.0), receiver, config,
                                        &double_power_link));
  EXPECT_NEAR(10.0 * std::log10(far_link.received_power_w / near_link.received_power_w), -6.0206,
              1.0e-3);
  EXPECT_NEAR(10.0 * std::log10(double_power_link.received_power_w / near_link.received_power_w),
              3.0103, 1.0e-3);
}

TEST(RfSceneTest, PropagationDelayDopplerAndPriorCycleArrivalAreExplicit) {
  RfSceneReceiverState receiver = MakeReceiver();
  receiver.window_start_time_s = 10.0;
  receiver.window_duration_s = 0.001;
  receiver.velocity_ecef_mps.x_mps = 300.0;
  RfSceneEmission emission = MakeNoiseEmission(1U, 299792.458, 4.0);
  ASSERT_TRUE(TryCreateRfNoiseWaveform(9.999, 0.001, 10.0e9, 20.0e6, 4.0, &emission.waveform));
  RfIncidentLinkResult link;
  ASSERT_TRUE(TryEvaluateRfIncidentLink(emission, receiver, RfIncidentLinkConfig{}, &link));
  EXPECT_NEAR(link.propagation_delay_s, 0.001, 1.0e-12);
  EXPECT_GT(link.doppler_shift_hz, 0.0);
  EXPECT_NEAR(link.arrival_start_time_s, 10.0, 1.0e-12);
  EXPECT_GT(link.received_power_w, 0.0);
}

TEST(RfSceneTest, PulseTrainAndSweepUseParameterizedTimeFrequencyOccupancy) {
  RfSceneReceiverState receiver = MakeReceiver();
  receiver.window_duration_s = 0.01;
  RfSceneEmission pulse = MakeNoiseEmission(1U, 1000.0, 100.0);
  ASSERT_TRUE(TryCreateRfPulseTrainWaveform(10.0, 10.0e9, 10.0e6, 100.0, 1.0e-4, 1.0e-3, 10U, 0.1,
                                            7U, 3U, &pulse.waveform));
  RfIncidentLinkResult pulse_link;
  ASSERT_TRUE(TryEvaluateRfIncidentLink(pulse, receiver, RfIncidentLinkConfig{}, &pulse_link));
  EXPECT_GT(pulse_link.time_overlap_fraction, 0.05);
  EXPECT_LT(pulse_link.time_overlap_fraction, 0.15);

  RfSceneEmission sweep = MakeNoiseEmission(2U, 1000.0, 100.0);
  ASSERT_TRUE(TryCreateRfLinearSweepWaveform(10.0, 0.01, 9.9e9, 10.1e9, 2.0e6, 100.0, 0.01,
                                             &sweep.waveform));
  RfIncidentLinkResult sweep_link;
  ASSERT_TRUE(TryEvaluateRfIncidentLink(sweep, receiver, RfIncidentLinkConfig{}, &sweep_link));
  EXPECT_GT(sweep_link.frequency_overlap_fraction, 0.05);
  EXPECT_LT(sweep_link.frequency_overlap_fraction, 0.20);
}

TEST(RfSceneTest, CoSiteRequiresDirectedEquipmentPath) {
  RfSceneReceiverState receiver = MakeReceiver();
  receiver.platform_id = 10U;
  receiver.equipment_id = 30U;
  RfSceneEmission emission = MakeNoiseEmission(1U, 0.0, 100.0);
  emission.identity.platform_id = 10U;
  emission.identity.equipment_id = 20U;
  emission.position_ecef_m = receiver.position_ecef_m;
  RfIncidentLinkResult untouched;
  untouched.received_power_w = 99.0;
  EXPECT_FALSE(TryEvaluateRfIncidentLink(emission, receiver, RfIncidentLinkConfig{}, &untouched));
  EXPECT_DOUBLE_EQ(untouched.received_power_w, 99.0);

  receiver.co_site_paths.push_back(RfCoSiteIsolationPath{20U, 30U, 80.0});
  RfIncidentLinkResult isolated;
  ASSERT_TRUE(TryEvaluateRfIncidentLink(emission, receiver, RfIncidentLinkConfig{}, &isolated));
  EXPECT_TRUE(isolated.is_co_site);
  EXPECT_NEAR(isolated.received_power_before_overlap_w, 1.0e-6, 1.0e-15);
}

TEST(RfSceneTest, ValidationAndAggregationFailClosedAndIgnoreInputOrder) {
  EXPECT_FALSE(TryCreateRfNoiseWaveform(10.0, 1.0, 10.0e9, 20.0e6, 10.0, nullptr));

  RfSceneFrame scene;
  scene.world_cycle_index = 1U;
  scene.window_start_time_s = 10.0;
  scene.window_duration_s = 1.0;
  scene.emissions.push_back(MakeNoiseEmission(1U, 1000.0, 10.0));
  scene.emissions.push_back(MakeNoiseEmission(2U, 2000.0, 10.0));
  EXPECT_TRUE(TryValidateRfSceneFrame(scene));
  scene.emissions[1].identity.emission_id = 1U;
  EXPECT_FALSE(TryValidateRfSceneFrame(scene));
  scene.emissions[1].identity.emission_id = 2U;
  scene.emissions[1].waveform.transmit_power_w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(TryValidateRfSceneFrame(scene));

  RfIncidentLinkResult first;
  first.identity = RfEmissionIdentity{10U, 20U, 1U};
  first.receiver_platform_id = 1U;
  first.receiver_equipment_id = 2U;
  first.received_power_w = 1.0;
  RfIncidentLinkResult second = first;
  second.identity.emission_id = 2U;
  second.received_power_w = 2.0;
  double forward = 0.0;
  double reverse = 0.0;
  ASSERT_TRUE(TryAggregateRfIncidentPower({first, second}, &forward));
  ASSERT_TRUE(TryAggregateRfIncidentPower({second, first}, &reverse));
  EXPECT_DOUBLE_EQ(forward, reverse);
  EXPECT_DOUBLE_EQ(forward, 3.0);
  double untouched_sum = 77.0;
  EXPECT_FALSE(TryAggregateRfIncidentPower({first, first}, &untouched_sum));
  EXPECT_DOUBLE_EQ(untouched_sum, 77.0);
}

}  // namespace
}  // namespace electromagnetics
}  // namespace oneq
