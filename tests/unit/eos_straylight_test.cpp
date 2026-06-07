/**
 * @file eos_straylight_unit_test.cpp
 * @brief 验证 EOS 遮光罩杂散光抑制模型与管线接入行为。
 */

#include <gtest/gtest.h>

#include "electro_optical_sensor/foundation/EosStrayLight.h"
#include "electro_optical_sensor/signal/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace foundation {
namespace stray_light {
namespace {

namespace context = ::electro_optical_sensor::session;

TEST(EosStrayLightTest, ContaminationReducesWhenSunTargetSeparationIncreases) {
  StrayLightFilterInputs near_sun_inputs;
  near_sun_inputs.enabled = true;
  near_sun_inputs.target_azimuth_deg = 175.0f;
  near_sun_inputs.target_elevation_deg = 44.0f;
  near_sun_inputs.sun_azimuth_deg = 180.0f;
  near_sun_inputs.sun_altitude_deg = 45.0f;

  StrayLightFilterInputs far_sun_inputs = near_sun_inputs;
  far_sun_inputs.target_azimuth_deg = 30.0f;
  far_sun_inputs.target_elevation_deg = 5.0f;

  const StrayLightFilterResult near_sun_result = EvaluateStrayLightFilter(near_sun_inputs);
  const StrayLightFilterResult far_sun_result = EvaluateStrayLightFilter(far_sun_inputs);

  EXPECT_GT(near_sun_result.contamination_ratio, far_sun_result.contamination_ratio);
  EXPECT_GT(near_sun_result.background_penalty_scale, far_sun_result.background_penalty_scale);
}

}  // namespace
}  // namespace stray_light
}  // namespace foundation
}  // namespace electro_optical_sensor

namespace electro_optical_sensor {
namespace signal {
namespace pipeline {
namespace context = ::electro_optical_sensor::session;
namespace {

session::EosSceneTarget MakeTarget(float azimuth_deg, float elevation_deg) {
  session::EosSceneTarget target;
  target.target_id = 1U;
  target.range_m = 500.0f;
  target.azimuth_deg = azimuth_deg;
  target.elevation_deg = elevation_deg;
  target.appearance.apparent_temperature_k = 950.0f;
  target.appearance.emissivity = 0.98f;
  target.appearance.reflectance = 0.70f;
  target.appearance.projected_area_m2 = 20.0f;
  return target;
}

::electro_optical_sensor::session::EosCycleInput MakeInput() {
  ::electro_optical_sensor::session::EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  input.environment.solar_azimuth_deg = 180.0f;
  input.environment.solar_altitude_deg = 45.0f;
  input.environment.solar_irradiance_w_m2 = 900.0f;
  input.environment.cloud_coverage_ratio = 0.0f;
  input.environment.background_temperature_k = 220.0f;
  input.environment.day_night_type = ::electro_optical_sensor::session::DayNightType::kDay;
  input.platform_altitude_m = 1200.0f;
  input.platform_pose.position_m.z = 0.0f;
  input.scene.push_back(MakeTarget(180.0f, 45.0f));
  return input;
}

EosPipelineConfig MakeConfig(bool enable_straylight_filter) {
  EosPipelineConfig config;
  config.mission.work_mode = EosPipelineWorkMode::kInfraredOnly;
  config.detection_policy.minimum_snr_db = -120.0f;
  config.mission.scan_start_az_deg = 170.0f;
  config.mission.scan_end_az_deg = 190.0f;
  config.mission.scan_center_el_deg = 45.0f;
  config.mission.scan_rate_deg_per_sec = 10.0f;
  config.mission.horizontal_fov_deg = 20.0f;
  config.mission.vertical_fov_deg = 20.0f;
  config.stray_light_policy.enable_straylight_filter = enable_straylight_filter;
  config.stray_light_policy.hood_inner_half_angle_deg = 8.0f;
  config.stray_light_policy.hood_outer_half_angle_deg = 70.0f;
  config.stray_light_policy.hood_min_suppression_ratio = 0.2f;
  config.stray_light_policy.hood_max_suppression_ratio = 0.9f;
  return config;
}

TEST(EosStrayLightPipelineTest, EnablingHoodFilterImprovesNearSunSnr) {
  EosPipeline pipeline_without_filter(MakeConfig(false));
  EosPipeline pipeline_with_filter(MakeConfig(true));
  const ::electro_optical_sensor::session::EosCycleInput input = MakeInput();

  const auto frame_without_filter = pipeline_without_filter.RunCycle(input);
  const auto frame_with_filter = pipeline_with_filter.RunCycle(input);

  ASSERT_EQ(frame_without_filter.detections.size(), 1U);
  ASSERT_EQ(frame_with_filter.detections.size(), 1U);
  EXPECT_GT(frame_with_filter.detections[0].fused_snr_linear,
            frame_without_filter.detections[0].fused_snr_linear);
}

}  // namespace
}  // namespace pipeline
}  // namespace signal
}  // namespace electro_optical_sensor
