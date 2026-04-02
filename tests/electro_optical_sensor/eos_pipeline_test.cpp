/**
 * @file eos_pipeline_test.cpp
 * @brief 验证 EOS core/pipeline 的扫描递推与视场判定策略。
 */

#include <gtest/gtest.h>

#include "electro_optical_sensor/core/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace core {
namespace pipeline {
namespace {

context::EosTargetState MakeTarget(std::uint64_t id, float azimuth_deg, float range_m = 1800.0f) {
  context::EosTargetState target;
  target.target_id = id;
  target.range_m = range_m;
  target.azimuth_deg = azimuth_deg;
  target.elevation_deg = 0.0f;
  target.apparent_temperature_k = 330.0f;
  target.emissivity = 0.92f;
  target.reflectance = 0.38f;
  target.projected_area_m2 = 2.0f;
  return target;
}

context::EosCycleInput MakeCycleInput(float dt_sec = 1.0f) {
  context::EosCycleInput input;
  input.dt_sec = dt_sec;
  input.solar_irradiance_w_m2 = 850.0f;
  input.solar_altitude_deg = 45.0f;
  input.atmospheric_transmittance = 0.8f;
  input.cloud_coverage_ratio = 0.2f;
  input.background_temperature_k = 289.0f;
  input.day_night_type = context::DayNightType::kDay;
  input.platform_pose.position_m.z = 1200.0f;
  return input;
}

EosPipelineConfig MakePipelineConfig() {
  EosPipelineConfig config;
  config.work_mode = EosPipelineWorkMode::kFused;
  config.minimum_snr_db = 0.0f;
  config.scan_start_az_deg = -10.0f;
  config.scan_end_az_deg = 10.0f;
  config.scan_rate_deg_per_sec = 5.0f;
  config.horizontal_fov_deg = 6.0f;
  config.vertical_fov_deg = 4.0f;
  return config;
}

TEST(EosPipelineTest, ScanAngleAdvancesAndWrapsInsideRange) {
  EosPipeline pipeline(MakePipelineConfig());
  context::EosCycleInput input = MakeCycleInput(5.0f);  // advance by 25 deg, wrap inside [-10, 10]
  input.cycle_index = 1U;
  input.scene_targets.push_back(MakeTarget(1U, -5.0f));

  const common::EosOutputFrame frame = pipeline.Execute(input);

  EXPECT_GE(frame.scan_azimuth_deg, -10.0f);
  EXPECT_LE(frame.scan_azimuth_deg, 10.0f);
}

TEST(EosPipelineTest, InFovTargetIsDetectedAndOutOfFovTargetIsFiltered) {
  EosPipeline pipeline(MakePipelineConfig());
  context::EosCycleInput input = MakeCycleInput(1.0f);
  input.cycle_index = 3U;
  input.scene_targets.push_back(MakeTarget(101U, -5.0f));   // in fov after first advance
  input.scene_targets.push_back(MakeTarget(102U, 35.0f));   // out of fov

  const common::EosOutputFrame frame = pipeline.Execute(input);

  ASSERT_EQ(frame.detections.size(), 1U);
  EXPECT_EQ(frame.detections[0].target_id, 101U);
}

TEST(EosPipelineTest, OutOfRangeTargetIsMarkedUndetected) {
  EosPipeline pipeline(MakePipelineConfig());
  context::EosCycleInput input = MakeCycleInput(1.0f);
  input.cycle_index = 4U;
  input.scene_targets.push_back(MakeTarget(201U, -5.0f, 8000.0f));

  const common::EosOutputFrame frame = pipeline.Execute(input);

  ASSERT_EQ(frame.detections.size(), 1U);
  EXPECT_FALSE(frame.detections[0].detected);
}

}  // namespace
}  // namespace pipeline
}  // namespace core
}  // namespace electro_optical_sensor
