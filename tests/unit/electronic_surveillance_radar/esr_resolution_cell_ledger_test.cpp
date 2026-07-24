#include <gtest/gtest.h>

#include <cmath>

#include "1q/electromagnetics/RfScene.h"
#include "electronic_surveillance_radar/pipeline/EsrResolutionCellLedger.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace {

oneq::electromagnetics::RfSceneReceiverState MakeReceiver() {
  oneq::electromagnetics::RfSceneReceiverState receiver;
  receiver.platform_id = 1U;
  receiver.equipment_id = 2U;
  receiver.window_start_time_s = 0.0;
  receiver.window_duration_s = 1.0;
  receiver.center_frequency_hz = 10.0e9;
  receiver.bandwidth_hz = 200.0e6;
  return receiver;
}

oneq::electromagnetics::RfIncidentLinkResult MakePulseLink(
    std::uint64_t emission_id, double first_pulse_time_s, double power_w) {
  oneq::electromagnetics::RfIncidentLinkResult link;
  link.identity.platform_id = 10U + emission_id;
  link.identity.equipment_id = 20U + emission_id;
  link.identity.emission_id = emission_id;
  link.receiver_platform_id = 1U;
  link.receiver_equipment_id = 2U;
  link.received_power_before_overlap_w = power_w;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfPulseTrainWaveform(
      first_pulse_time_s, 10.0e9, 1.0e6, 1.0, 1.0e-3, 0.1, 1U, 0.0,
      emission_id, 0U, &link.emission_waveform));
  return link;
}

EsrArrivalBearing MakeBearing(double azimuth_deg) {
  EsrArrivalBearing bearing;
  bearing.defined = true;
  bearing.azimuth_deg = azimuth_deg;
  return bearing;
}

EsrArrivalBearing MakeBearingWithoutObservableAzimuth(double elevation_deg) {
  EsrArrivalBearing bearing;
  bearing.defined = true;
  bearing.azimuth_observable = false;
  bearing.elevation_deg = elevation_deg;
  return bearing;
}

TEST(EsrResolutionCellLedgerTest, TimeSeparatedPulsesDoNotInterfere) {
  std::vector<oneq::electromagnetics::RfIncidentLinkResult> links;
  links.push_back(MakePulseLink(1U, 0.1, 2.0));
  links.push_back(MakePulseLink(2U, 0.6, 1.0));
  const std::vector<EsrArrivalBearing> bearings{MakeBearing(0.0),
                                                MakeBearing(0.0)};

  EsrResolutionCellLedgerResult result;
  ASSERT_TRUE(TryBuildEsrResolutionCellLedger(links, bearings, MakeReceiver(),
                                               5.0, &result));
  ASSERT_EQ(result.candidates.size(), 2U);
  EXPECT_DOUBLE_EQ(result.candidates[0].interference_power_w, 0.0);
  EXPECT_DOUBLE_EQ(result.candidates[1].interference_power_w, 0.0);
}

TEST(EsrResolutionCellLedgerTest, SameCellPublishesStrongestAndBooksOtherAsInterference) {
  std::vector<oneq::electromagnetics::RfIncidentLinkResult> links;
  links.push_back(MakePulseLink(1U, 0.1, 2.0));
  links.push_back(MakePulseLink(2U, 0.1, 1.0));
  const std::vector<EsrArrivalBearing> bearings{MakeBearing(0.0),
                                                MakeBearing(0.0)};

  EsrResolutionCellLedgerResult result;
  ASSERT_TRUE(TryBuildEsrResolutionCellLedger(links, bearings, MakeReceiver(),
                                               5.0, &result));
  ASSERT_EQ(result.candidates.size(), 1U);
  EXPECT_EQ(result.candidates[0].source_index, 0U);
  EXPECT_GT(result.candidates[0].interference_power_w, 0.0);
}

TEST(EsrResolutionCellLedgerTest,
     PolarBearingWithoutObservableAzimuthCannotBecomeCandidate) {
  std::vector<oneq::electromagnetics::RfIncidentLinkResult> links;
  links.push_back(MakePulseLink(1U, 0.1, 2.0));
  links.push_back(MakePulseLink(2U, 0.1, 1.0));
  EsrArrivalBearing observable_polar = MakeBearing(0.0);
  observable_polar.elevation_deg = 90.0;
  const std::vector<EsrArrivalBearing> bearings{
      MakeBearingWithoutObservableAzimuth(90.0), observable_polar};

  EsrResolutionCellLedgerResult result;
  ASSERT_TRUE(TryBuildEsrResolutionCellLedger(links, bearings, MakeReceiver(),
                                               180.0, &result));
  ASSERT_EQ(result.candidates.size(), 1U);
  EXPECT_EQ(result.candidates[0].source_index, 1U);
  EXPECT_GT(result.candidates[0].interference_power_w, 0.0);
}

TEST(EsrResolutionCellLedgerTest, LinearSweepUsesPartialInstantaneousChannelDwell) {
  oneq::electromagnetics::RfIncidentLinkResult link;
  link.identity.platform_id = 10U;
  link.identity.equipment_id = 20U;
  link.identity.emission_id = 1U;
  link.receiver_platform_id = 1U;
  link.receiver_equipment_id = 2U;
  link.received_power_before_overlap_w = 1.0;
  ASSERT_TRUE(oneq::electromagnetics::TryCreateRfLinearSweepWaveform(
      0.0, 1.0, 9.0e9, 11.0e9, 1.0e6, 1.0, 1.0,
      &link.emission_waveform));
  EsrResolutionCellLedgerResult result;
  ASSERT_TRUE(TryBuildEsrResolutionCellLedger(
      std::vector<oneq::electromagnetics::RfIncidentLinkResult>{link},
      std::vector<EsrArrivalBearing>{MakeBearing(0.0)}, MakeReceiver(), 5.0,
      &result));

  ASSERT_EQ(result.candidates.size(), 1U);
  EXPECT_GT(result.candidates[0].signal_power_w, 0.08);
  EXPECT_LT(result.candidates[0].signal_power_w, 0.12);
  EXPECT_NEAR(result.candidates[0].estimated_center_frequency_hz, 10.0e9,
              5.0e6);
}

}  // namespace
}  // namespace pipeline
}  // namespace electronic_surveillance_radar
