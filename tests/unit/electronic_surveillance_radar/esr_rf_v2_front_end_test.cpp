#include <gtest/gtest.h>

#include "1q/electromagnetics/RfScene.h"
#include "electronic_surveillance_radar/pipeline/EsrRfV2FrontEnd.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace {

session::EsrCycleInput MakeInput() {
  session::EsrCycleInput input;
  input.cycle_index = 9U;
  input.cycle_start_time_s = 10.0;
  input.dt_sec = 1.0f;
  input.platform_entity_id = 1U;
  input.has_platform_ecef_kinematics = true;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.rf_emissions.world_cycle_index = input.cycle_index;
  input.rf_emissions.window_start_time_s = input.cycle_start_time_s;
  input.rf_emissions.window_duration_s = input.dt_sec;
  return input;
}

oneq::electromagnetics::RfSceneEmission MakeEmission(std::uint64_t emission_id,
                                                      double x_offset_m, double power_w) {
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.platform_id = 10U + emission_id;
  emission.identity.equipment_id = 20U + emission_id;
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m.x_m = 6378137.0 + x_offset_m;
  emission.antenna.boresight_ecef.x = -1.0;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(
      10.0, 1.0, 10.0e9, 2.0e6, power_w, &emission.waveform));
  return emission;
}

config::EsrHardwareConfig MakeHardware() {
  config::EsrHardwareConfig hardware;
  hardware.receiver_equipment_id = 2U;
  hardware.receiver_band_lower_hz = 1.0e9;
  hardware.receiver_band_upper_hz = 18.0e9;
  hardware.maximum_linear_input_power_w = 1.0e-30f;
  return hardware;
}

TEST(EsrRfV2FrontEndTest, ResolvesEveryEmissionOnceAndAggregatesInStableOrder) {
  session::EsrCycleInput input = MakeInput();
  input.rf_emissions.emissions.push_back(MakeEmission(2U, 2000.0, 10.0));
  input.rf_emissions.emissions.push_back(MakeEmission(1U, 1000.0, 10.0));

  EsrRfV2FrontEndResult result;
  ASSERT_TRUE(TryResolveEsrRfV2FrontEnd(input, MakeHardware(), 0.0, 0.0, 10.0e9, 2.0e6, 0.0,
                                        &result));
  ASSERT_EQ(result.front_end_incident_links.size(), 2U);
  ASSERT_EQ(result.channel_incident_links.size(), 2U);
  EXPECT_EQ(result.front_end_incident_links[0].identity.emission_id, 1U);
  EXPECT_EQ(result.channel_incident_links[1].identity.emission_id, 2U);
  EXPECT_GT(result.total_incident_power_w, 0.0);
  EXPECT_TRUE(result.receiver_saturated);
}

TEST(EsrRfV2FrontEndTest, StrongHardwareBandSignalOutsideTunedChannelStillSaturates) {
  session::EsrCycleInput input = MakeInput();
  input.rf_emissions.emissions.push_back(MakeEmission(1U, 1000.0, 10.0));
  input.rf_emissions.emissions.back().waveform.center_frequency_hz = 10.1e9;

  config::EsrHardwareConfig hardware = MakeHardware();
  hardware.receiver_band_lower_hz = 9.0e9;
  hardware.receiver_band_upper_hz = 11.0e9;
  EsrRfV2FrontEndResult result;
  ASSERT_TRUE(TryResolveEsrRfV2FrontEnd(input, hardware, 0.0, 0.0, 10.0e9, 2.0e6, 0.0,
                                        &result));
  ASSERT_EQ(result.front_end_incident_links.size(), 1U);
  ASSERT_EQ(result.channel_incident_links.size(), 1U);
  EXPECT_GT(result.front_end_incident_links[0].received_power_w, 0.0);
  EXPECT_EQ(result.channel_incident_links[0].received_power_w, 0.0);
  EXPECT_TRUE(result.receiver_saturated);
}

TEST(EsrRfV2FrontEndTest, RejectsUnmatchedFrameBeforeProducingResult) {
  session::EsrCycleInput input = MakeInput();
  input.rf_emissions.window_start_time_s += 0.1;
  EsrRfV2FrontEndResult result;
  EXPECT_FALSE(TryResolveEsrRfV2FrontEnd(input, MakeHardware(), 0.0, 0.0, 10.0e9, 2.0e6, 0.0,
                                         &result));
}

TEST(EsrRfV2FrontEndTest, RequiresEquipmentSpecificCoSiteIsolation) {
  session::EsrCycleInput input = MakeInput();
  oneq::electromagnetics::RfSceneEmission emission = MakeEmission(1U, 0.0, 10.0);
  emission.identity.platform_id = input.platform_entity_id;
  emission.identity.equipment_id = 99U;
  input.rf_emissions.emissions.push_back(emission);
  EsrRfV2FrontEndResult result;
  EXPECT_FALSE(TryResolveEsrRfV2FrontEnd(input, MakeHardware(), 0.0, 0.0, 10.0e9, 2.0e6, 0.0,
                                         &result));

  config::EsrHardwareConfig hardware = MakeHardware();
  hardware.co_site_paths.push_back(config::EsrCoSiteIsolationPath{99U, 80.0});
  EXPECT_TRUE(TryResolveEsrRfV2FrontEnd(input, hardware, 0.0, 0.0, 10.0e9, 2.0e6, 0.0,
                                        &result));
}

}  // namespace
}  // namespace pipeline
}  // namespace electronic_surveillance_radar
