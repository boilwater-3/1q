#include <gtest/gtest.h>

#include <algorithm>

#include "airborne_radar/signal/detection/ArInterferenceObservationResolver.h"

namespace airborne_radar {
namespace signal {
namespace detection {
namespace {

oneq::electromagnetics::RfSceneEmission MakeJammer(std::uint64_t emission_id, double y_m,
                                                   double power_w) {
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity = oneq::electromagnetics::RfEmissionIdentity{10U, 20U, emission_id};
  emission.position_ecef_m.x_m = 1000.0;
  emission.position_ecef_m.y_m = y_m;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(10.0, 1.0, 10.0e9, 20.0e6, power_w,
                                                               &emission.waveform));
  return emission;
}

oneq::electromagnetics::RfIncidentLinkResult MakeLink(
    const oneq::electromagnetics::RfSceneEmission& emission, double power_w) {
  oneq::electromagnetics::RfIncidentLinkResult link;
  link.identity = emission.identity;
  link.received_power_w = power_w;
  return link;
}

TEST(ArInterferenceObservationResolverTest, GatesByJOverNAndIsOrderIndependent) {
  oneq::electromagnetics::RfSceneFrame scene;
  scene.world_cycle_index = 1U;
  scene.window_start_time_s = 10.0;
  scene.window_duration_s = 1.0;
  scene.emissions = {MakeJammer(2U, 100.0, 10.0), MakeJammer(1U, -100.0, 10.0)};
  oneq::electromagnetics::RfSceneReceiverState receiver;
  receiver.platform_id = 1U;
  receiver.equipment_id = 2U;
  receiver.antenna.half_power_beamwidth_deg = 4.0;
  const std::vector<oneq::electromagnetics::RfIncidentLinkResult> links = {
      MakeLink(scene.emissions[0], 100.0), MakeLink(scene.emissions[1], 0.01)};

  std::vector<session::ArInterferenceObservation> forward;
  ASSERT_TRUE(TryResolveArInterferenceObservations(
      scene, receiver, oneq::electromagnetics::RfEmissionIdentity{1U, 3U, 4U}, links, 1.0, 0.0,
      &forward));
  ASSERT_EQ(forward.size(), 1U);
  EXPECT_EQ(forward.front().observation_id, 1U);
  EXPECT_GT(forward.front().jammer_to_noise_db, 0.0);

  std::reverse(scene.emissions.begin(), scene.emissions.end());
  auto reversed_links = links;
  std::reverse(reversed_links.begin(), reversed_links.end());
  std::vector<session::ArInterferenceObservation> reverse;
  ASSERT_TRUE(TryResolveArInterferenceObservations(
      scene, receiver, oneq::electromagnetics::RfEmissionIdentity{1U, 3U, 4U}, reversed_links, 1.0,
      0.0, &reverse));
  ASSERT_EQ(reverse.size(), forward.size());
  EXPECT_DOUBLE_EQ(reverse.front().estimated_bearing_azimuth_deg,
                   forward.front().estimated_bearing_azimuth_deg);
  EXPECT_EQ(reverse.front().observation_id, forward.front().observation_id);
}

TEST(ArInterferenceObservationResolverTest, UncertaintyDecreasesWithJOverN) {
  oneq::electromagnetics::RfSceneFrame scene;
  scene.world_cycle_index = 1U;
  scene.window_start_time_s = 10.0;
  scene.window_duration_s = 1.0;
  scene.emissions.push_back(MakeJammer(1U, 0.0, 10.0));
  oneq::electromagnetics::RfSceneReceiverState receiver;
  receiver.platform_id = 1U;
  receiver.equipment_id = 2U;
  receiver.antenna.half_power_beamwidth_deg = 4.0;

  std::vector<session::ArInterferenceObservation> low;
  std::vector<session::ArInterferenceObservation> high;
  ASSERT_TRUE(TryResolveArInterferenceObservations(
      scene, receiver, oneq::electromagnetics::RfEmissionIdentity{1U, 3U, 4U},
      {MakeLink(scene.emissions.front(), 10.0)}, 1.0, 0.0, &low));
  ASSERT_TRUE(TryResolveArInterferenceObservations(
      scene, receiver, oneq::electromagnetics::RfEmissionIdentity{1U, 3U, 4U},
      {MakeLink(scene.emissions.front(), 100.0)}, 1.0, 0.0, &high));
  ASSERT_EQ(low.size(), 1U);
  ASSERT_EQ(high.size(), 1U);
  EXPECT_LT(high.front().bearing_standard_deviation_deg,
            low.front().bearing_standard_deviation_deg);
  EXPECT_LT(high.front().frequency_standard_deviation_hz,
            low.front().frequency_standard_deviation_hz);
}

}  // namespace
}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar
