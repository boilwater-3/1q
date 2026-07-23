#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>

#include "1q/electromagnetics/RfScene.h"
#include "electronic_surveillance_radar/pipeline/InterceptDetectionExecutor.h"
#include "electronic_surveillance_radar/pipeline/MutableEsrContext.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace {

session::EsrCycleInput MakeInput() {
  session::EsrCycleInput input;
  input.cycle_index = 4U;
  input.cycle_start_time_s = 10.0;
  input.dt_sec = 1.0f;
  input.platform_entity_id = 1U;
  input.has_platform_ecef_kinematics = true;
  input.platform_position_ecef_m.x_m = 6378137.0;
  input.interference.world_cycle_index = input.cycle_index;
  input.interference.window_start_time_s = input.cycle_start_time_s;
  input.interference.window_duration_s = input.dt_sec;
  return input;
}

oneq::electromagnetics::RfSceneEmission MakeEmission(std::uint64_t emission_id, double center_hz,
                                                     double power_w) {
  oneq::electromagnetics::RfSceneEmission emission;
  emission.identity.platform_id = 10U + emission_id;
  emission.identity.equipment_id = 20U + emission_id;
  emission.identity.emission_id = emission_id;
  emission.position_ecef_m.x_m = 6378137.0;
  emission.position_ecef_m.y_m = 1000.0;
  emission.antenna.boresight_ecef.y = -1.0;
  EXPECT_TRUE(oneq::electromagnetics::TryCreateRfNoiseWaveform(10.0, 1.0, center_hz, 1.0e6, power_w,
                                                               &emission.waveform));
  return emission;
}

InterceptDetectionOutput RunDetection(const session::EsrCycleInput& input,
                                      float maximum_linear_input_power_w = 10.0f,
                                      std::uint64_t completed_receive_cycles = 0U,
                                      bool use_tuning_plan = false,
                                      bool enable_statistical_detection = false,
                                      float beamwidth_deg = 120.0f) {
  extension::InterceptPipelineConfig pipeline_config;
  pipeline_config.detection.minimum_snr_db = -20.0f;
  pipeline_config.statistical_detection.enable_statistical_detection = enable_statistical_detection;
  pipeline_config.scan.scan_start_az_deg = 0.0f;
  pipeline_config.scan.scan_end_az_deg = 0.0f;
  pipeline_config.scan.scan_start_el_deg = 0.0f;
  pipeline_config.scan.scan_end_el_deg = 0.0f;

  extension::InterceptRuntimeConfig runtime_config;
  runtime_config.receiver_hardware.receiver_equipment_id = 2U;
  runtime_config.receiver_hardware.receiver_band_lower_hz = 9.99e9;
  runtime_config.receiver_hardware.receiver_band_upper_hz = 10.01e9;
  runtime_config.receiver_hardware.beam_az_width_deg = beamwidth_deg;
  runtime_config.receiver_hardware.beam_el_width_deg = beamwidth_deg;
  runtime_config.receiver_hardware.maximum_linear_input_power_w = maximum_linear_input_power_w;
  if (use_tuning_plan) {
    config::EsrTuningWindow first_window;
    first_window.center_frequency_hz = 9.5e9;
    first_window.bandwidth_hz = 1.0e6;
    first_window.dwell_cycles = 1U;
    config::EsrTuningWindow second_window;
    second_window.center_frequency_hz = 10.0e9;
    second_window.bandwidth_hz = 1.0e6;
    second_window.dwell_cycles = 1U;
    runtime_config.receiver_hardware.tuning_plan = {first_window, second_window};
  }

  session::EsrEnvironmentSnapshot environment;
  MutableEsrContext context;
  context.BeginCycle(input, environment, pipeline_config, runtime_config);
  InterceptDetectionExecutor executor;
  std::uint64_t next_observation_id = 1U;
  double scan_phase_cycles = 0.0;
  return executor.Execute(context, next_observation_id, &scan_phase_cycles,
                          completed_receive_cycles);
}

TEST(EsrRfV2DetectionTest, EmitsDeclassifiedObservationFromRfFrame) {
  session::EsrCycleInput input = MakeInput();
  input.interference.emissions.push_back(MakeEmission(1U, 10.0e9, 1.0e6));

  const InterceptDetectionOutput output = RunDetection(input);
  ASSERT_EQ(output.raw_records.size(), 1U);
  const RawObservationRecord& record = output.raw_records.front();
  EXPECT_NEAR(record.observation.rf_hz, 10.0e9, 1.0);
  EXPECT_GT(record.observation.bandwidth_hz, 0.0);
  EXPECT_GT(record.observation.snr_db, -20.0);
}

TEST(EsrRfV2DetectionTest, SameChannelEmissionReducesSnrWithoutBooleanPenalty) {
  session::EsrCycleInput baseline_input = MakeInput();
  baseline_input.interference.emissions.push_back(MakeEmission(1U, 10.0e9, 1.0e6));
  const InterceptDetectionOutput baseline = RunDetection(baseline_input);
  ASSERT_EQ(baseline.raw_records.size(), 1U);

  session::EsrCycleInput interfered_input = baseline_input;
  interfered_input.interference.emissions.push_back(MakeEmission(2U, 10.0e9, 1.0e8));
  const InterceptDetectionOutput interfered = RunDetection(interfered_input);
  ASSERT_EQ(interfered.raw_records.size(), 2U);
  EXPECT_LT(interfered.raw_records.front().observation.snr_db,
            baseline.raw_records.front().observation.snr_db);
}

TEST(EsrRfV2DetectionTest, AngularlyResolvedSameFrequencySourceDoesNotEnterInterferenceCell) {
  session::EsrCycleInput baseline_input = MakeInput();
  baseline_input.interference.emissions.push_back(MakeEmission(1U, 10.0e9, 1.0e6));
  const InterceptDetectionOutput baseline =
      RunDetection(baseline_input, 10.0f, 0U, false, false, 10.0f);
  ASSERT_EQ(baseline.raw_records.size(), 1U);

  session::EsrCycleInput separated_input = baseline_input;
  oneq::electromagnetics::RfSceneEmission separated = MakeEmission(2U, 10.0e9, 1.0e8);
  separated.position_ecef_m.x_m += 1000.0;
  separated.position_ecef_m.y_m = 1000.0;
  separated.antenna.boresight_ecef.x = -1.0;
  separated.antenna.boresight_ecef.y = -1.0;
  separated_input.interference.emissions.push_back(separated);
  const InterceptDetectionOutput separated_output =
      RunDetection(separated_input, 10.0f, 0U, false, false, 10.0f);
  ASSERT_EQ(separated_output.raw_records.size(), 2U);
  EXPECT_DOUBLE_EQ(separated_output.raw_records.front().observation.snr_db,
                   baseline.raw_records.front().observation.snr_db);
}

TEST(EsrRfV2DetectionTest, EmissionOrderDoesNotChangeSemanticRandomMeasurements) {
  session::EsrCycleInput forward = MakeInput();
  forward.interference.emissions.push_back(MakeEmission(2U, 10.0e9, 1.0e6));
  forward.interference.emissions.push_back(MakeEmission(1U, 10.0e9, 1.0e6));
  session::EsrCycleInput reverse = forward;
  std::reverse(reverse.interference.emissions.begin(), reverse.interference.emissions.end());

  const InterceptDetectionOutput forward_output = RunDetection(forward, 10.0f, 0U, false, true);
  const InterceptDetectionOutput reverse_output = RunDetection(reverse, 10.0f, 0U, false, true);
  ASSERT_EQ(forward_output.raw_records.size(), reverse_output.raw_records.size());
  for (std::size_t index = 0U; index < forward_output.raw_records.size(); ++index) {
    const session::EmitterObservation& left = forward_output.raw_records[index].observation;
    const session::EmitterObservation& right = reverse_output.raw_records[index].observation;
    EXPECT_EQ(left.observation_id, right.observation_id);
    EXPECT_DOUBLE_EQ(left.aoa_az_deg, right.aoa_az_deg);
    EXPECT_DOUBLE_EQ(left.aoa_el_deg, right.aoa_el_deg);
    EXPECT_DOUBLE_EQ(left.snr_db, right.snr_db);
  }
}

TEST(EsrRfV2DetectionTest, SaturationCompletesWithoutFabricatedObservation) {
  session::EsrCycleInput input = MakeInput();
  input.interference.emissions.push_back(MakeEmission(1U, 10.0e9, 1.0e12));
  const InterceptDetectionOutput output = RunDetection(input, 1.0e-4f);
  EXPECT_TRUE(output.receiver_saturated);
  EXPECT_TRUE(output.raw_records.empty());
}

TEST(EsrRfV2DetectionTest, TuningPlanUsesCompletedReceiveCyclesRatherThanInputCycleIndex) {
  session::EsrCycleInput input = MakeInput();
  input.cycle_index = 999U;
  input.interference.world_cycle_index = input.cycle_index;

  const InterceptDetectionOutput initial = RunDetection(input, 10.0f, 0U, true);
  const InterceptDetectionOutput after_one_completed = RunDetection(input, 10.0f, 1U, true);
  EXPECT_DOUBLE_EQ(initial.receiver_center_frequency_hz, 9.5e9);
  EXPECT_DOUBLE_EQ(after_one_completed.receiver_center_frequency_hz, 10.0e9);
}

TEST(EsrRfV2DetectionTest, MissingCoSitePathRejectsTheV2CycleBeforeProducingObservation) {
  session::EsrCycleInput input = MakeInput();
  oneq::electromagnetics::RfSceneEmission emission = MakeEmission(1U, 10.0e9, 1.0e6);
  emission.identity.platform_id = input.platform_entity_id;
  emission.identity.equipment_id = 99U;
  input.interference.emissions.push_back(emission);

  const InterceptDetectionOutput output = RunDetection(input);
  EXPECT_TRUE(output.rf_v2_rejected);
  EXPECT_TRUE(output.raw_records.empty());
}

}  // namespace
}  // namespace pipeline
}  // namespace electronic_surveillance_radar
