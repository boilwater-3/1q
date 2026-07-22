/**
 * @file esr_rf_interference_test.cpp
 * @brief 验证 ESR 工程 RF 干扰链路、受扰判定与线性接收机边界。
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <random>

#include "1q/electromagnetics/RfLinkBudget.h"
#include "electronic_surveillance_radar/pipeline/InterceptDetectionExecutor.h"
#include "electronic_surveillance_radar/pipeline/MutableEsrContext.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace {

session::EsrSceneEmitter MakeTarget() {
  session::EsrSceneEmitter emitter;
  emitter.emitter_id = 101U;
  emitter.has_ecef_kinematics = true;
  emitter.position_ecef_m.x_m = 1000.0;
  emitter.pose.position_m.x = 1000.0f;
  emitter.carrier_hz = 10.0e9;
  emitter.bandwidth_hz = 1.0e6;
  emitter.tx_power_w = 1.0e6;
  emitter.pulse_width_s = 5.0e-4;
  emitter.pri_s = 1.0e-3;
  return emitter;
}

oneq::electromagnetics::RfEmission MakeEngineeringEmission(double center_hz, double power_w) {
  oneq::electromagnetics::RfEmission emission;
  emission.emission_id = 201U;
  emission.entity_id = 201U;
  emission.position_ecef_m.x_m = 2000.0;
  emission.antenna.boresight_ecef_unit.x = -1.0;
  oneq::electromagnetics::RfEmissionSegment segment;
  segment.duration_s = 1.0;
  segment.center_frequency_hz = center_hz;
  segment.bandwidth_hz = 1.0e6;
  segment.transmit_power_w = power_w;
  emission.segments.push_back(segment);
  return emission;
}

InterceptDetectionOutput RunDetection(const session::EsrEnvironmentSnapshot& environment,
                                      float maximum_linear_input_power_w,
                                      bool add_nonoverlapping_scene_emitter = false) {
  session::EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.platform_entity_id = 10U;
  input.has_platform_ecef_kinematics = true;
  input.scene.push_back(MakeTarget());
  if (add_nonoverlapping_scene_emitter) {
    session::EsrSceneEmitter other = MakeTarget();
    other.emitter_id = 102U;
    other.carrier_hz = 10.004e9;
    input.scene.push_back(other);
  }

  extension::InterceptPipelineConfig pipeline_config;
  pipeline_config.detection.receiver_noise_floor_w = 1.0e-12f;
  pipeline_config.detection.minimum_snr_db = -20.0f;
  pipeline_config.detection.max_detect_range_m = 100000.0f;
  pipeline_config.detection.min_dynamic_range_margin_db = -100.0f;
  pipeline_config.statistical_detection.enable_statistical_detection = false;
  pipeline_config.scan.scan_start_az_deg = 0.0f;
  pipeline_config.scan.scan_end_az_deg = 0.0f;
  pipeline_config.scan.scan_start_el_deg = 0.0f;
  pipeline_config.scan.scan_end_el_deg = 0.0f;
  pipeline_config.scan.az_step_deg = 5.0f;
  pipeline_config.scan.el_step_deg = 5.0f;

  extension::InterceptRuntimeConfig runtime_config;
  runtime_config.receiver_hardware.receiver_band_lower_hz = 9.995e9;
  runtime_config.receiver_hardware.receiver_band_upper_hz = 10.005e9;
  runtime_config.receiver_hardware.beam_az_width_deg = 120.0f;
  runtime_config.receiver_hardware.beam_el_width_deg = 120.0f;
  runtime_config.receiver_hardware.maximum_linear_input_power_w =
      maximum_linear_input_power_w;
  runtime_config.receiver_hardware.jamming_jn_threshold_db = 3.0f;
  runtime_config.receiver_hardware.jamming_snr_loss_threshold_db = 3.0f;

  MutableEsrContext context;
  context.BeginCycle(input, environment, pipeline_config, runtime_config);
  InterceptDetectionExecutor executor;
  std::mt19937 rng(42U);
  std::uint64_t next_observation_id = 1U;
  double scan_phase_cycles = 0.0;
  return executor.Execute(context, rng, next_observation_id, &scan_phase_cycles);
}

TEST(EsrRfInterferenceTest, SameFrequencyEmissionLowersSnrWithoutBooleanQualityPenalty) {
  session::EsrEnvironmentSnapshot baseline_environment;
  const InterceptDetectionOutput baseline = RunDetection(baseline_environment, 1.0f);
  ASSERT_EQ(baseline.raw_records.size(), 1U);

  session::EsrEnvironmentSnapshot jammed_environment;
  jammed_environment.interference_mode =
      oneq::electromagnetics::RfInterferenceMode::kEngineering;
  jammed_environment.engineering_emissions.push_back(
      MakeEngineeringEmission(10.0e9, 1.0e4));
  const InterceptDetectionOutput jammed = RunDetection(jammed_environment, 1.0f);

  ASSERT_EQ(jammed.raw_records.size(), 1U);
  EXPECT_LT(jammed.raw_records.front().observation.snr_db,
            baseline.raw_records.front().observation.snr_db);
  EXPECT_TRUE(jammed.raw_records.front().observation.is_jammed);
  EXPECT_EQ(jammed.raw_records.front().observation.quality,
            session::EsrObservationQuality::kHigh);
}

TEST(EsrRfInterferenceTest, OutOfBandEmissionContributesNoInterference) {
  session::EsrEnvironmentSnapshot baseline_environment;
  const InterceptDetectionOutput baseline = RunDetection(baseline_environment, 1.0f);

  session::EsrEnvironmentSnapshot out_of_band_environment;
  out_of_band_environment.interference_mode =
      oneq::electromagnetics::RfInterferenceMode::kEngineering;
  out_of_band_environment.engineering_emissions.push_back(
      MakeEngineeringEmission(11.0e9, 1.0e12));
  const InterceptDetectionOutput out_of_band = RunDetection(out_of_band_environment, 1.0f);

  ASSERT_EQ(baseline.raw_records.size(), 1U);
  ASSERT_EQ(out_of_band.raw_records.size(), 1U);
  EXPECT_DOUBLE_EQ(out_of_band.raw_records.front().observation.snr_db,
                   baseline.raw_records.front().observation.snr_db);
  EXPECT_FALSE(out_of_band.raw_records.front().observation.is_jammed);
}

TEST(EsrRfInterferenceTest, SceneEmitterOutsideSignalChannelDoesNotReduceTargetSnr) {
  session::EsrEnvironmentSnapshot environment;
  const InterceptDetectionOutput baseline = RunDetection(environment, 1.0f);
  const InterceptDetectionOutput with_other = RunDetection(environment, 1.0f, true);

  ASSERT_EQ(baseline.raw_records.size(), 1U);
  const RawObservationRecord* target_record = nullptr;
  for (const RawObservationRecord& record : with_other.raw_records) {
    if (record.truth_emitter_id == 101U) {
      target_record = &record;
    }
  }
  ASSERT_NE(target_record, nullptr);
  EXPECT_DOUBLE_EQ(target_record->observation.snr_db,
                   baseline.raw_records.front().observation.snr_db);
  EXPECT_FALSE(target_record->observation.is_jammed);
}

TEST(EsrRfInterferenceTest, SaturationProducesStatusAndNoObservation) {
  session::EsrEnvironmentSnapshot environment;
  environment.interference_mode = oneq::electromagnetics::RfInterferenceMode::kEngineering;
  environment.engineering_emissions.push_back(MakeEngineeringEmission(10.0e9, 1.0e12));

  const InterceptDetectionOutput output = RunDetection(environment, 1.0e-4f);
  EXPECT_TRUE(output.receiver_saturated);
  EXPECT_TRUE(output.raw_records.empty());
}

}  // namespace
}  // namespace pipeline
}  // namespace electronic_surveillance_radar
