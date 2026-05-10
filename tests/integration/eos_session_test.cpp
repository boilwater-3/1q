/**
 * @file eos_session_test.cpp
 * @brief 验证 EOS Session 单周期闭环：三工作模式输出、环境退化、运行时热切换。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/environment/EosEnvironmentTypes.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosSession.h"

namespace electro_optical_sensor {
namespace session {
namespace {

namespace context = ::electro_optical_sensor::session;
namespace output = ::electro_optical_sensor::output;

namespace eos_config = ::electro_optical_sensor::config;

session::EosSceneTarget MakeTarget(std::uint64_t id, float azimuth_deg, float range_m = 1800.0f,
                                   float projected_area_m2 = 2.0f) {
  session::EosSceneTarget target;
  target.target_id = id;
  target.range_m = range_m;
  target.azimuth_deg = azimuth_deg;
  target.elevation_deg = 0.0f;
  target.appearance.apparent_temperature_k = 330.0f;
  target.appearance.emissivity = 0.92f;
  target.appearance.reflectance = 0.38f;
  target.appearance.projected_area_m2 = projected_area_m2;
  return target;
}

::electro_optical_sensor::session::EosCycleInput MakeBaseInput() {
  ::electro_optical_sensor::session::EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  input.environment.solar_irradiance_w_m2 = 850.0f;
  input.environment.solar_altitude_deg = 45.0f;
  input.environment.cloud_coverage_ratio = 0.2f;
  input.environment.background_temperature_k = 289.0f;
  input.environment.day_night_type = ::electro_optical_sensor::session::DayNightType::kDay;
  input.platform_altitude_m = 1200.0f;
  input.platform_pose.position_m.z = 0.0f;
  input.scene.push_back(MakeTarget(1U, 0.0f, 1500.0f, 4.0f));
  return input;
}

EosSessionConfig MakeSessionConfig() {
  EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kFused;
  config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  config.mission.scan_start_az_deg = -10.0f;
  config.mission.scan_end_az_deg = 10.0f;
  config.mission.scan_rate_deg_per_sec = 5.0f;
  config.mission.horizontal_fov_deg = 20.0f;
  config.mission.vertical_fov_deg = 4.0f;
  return config;
}

std::size_t CountDetectedTargets(const session::EosOutputFrame& frame) {
  std::size_t count = 0U;
  for (std::size_t i = 0; i < frame.detections.size(); ++i) {
    if (frame.detections[i].detected) {
      ++count;
    }
  }
  return count;
}

bool HasTargetId(const session::EosOutputFrame& frame, std::uint64_t target_id) {
  for (std::size_t i = 0; i < frame.detections.size(); ++i) {
    if (frame.detections[i].target_id == target_id) {
      return true;
    }
  }
  return false;
}

const output::EosDetectionRecord* FindDetection(const session::EosOutputFrame& frame,
                                                std::uint64_t target_id) {
  for (std::size_t i = 0; i < frame.detections.size(); ++i) {
    if (frame.detections[i].target_id == target_id) {
      return &frame.detections[i];
    }
  }
  return nullptr;
}

TEST(EosSessionIntegrationTest, StepWithResultProducesDetectionOutput) {
  EosSession session = EosSessionFactory::Create(MakeSessionConfig());
  const ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_EQ(result.output_frame.cycle_index, input.cycle_index);
  EXPECT_FALSE(result.output_frame.detections.empty());
}

TEST(EosSessionIntegrationTest, StepReturnsSameCycleIndexAsStepWithResult) {
  EosSession session = EosSessionFactory::Create(MakeSessionConfig());
  const ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();

  const session::EosOutputFrame step_frame = session.Step(input);
  const ::electro_optical_sensor::session::EosCycleResult result =
      session.StepWithResult(MakeBaseInput());

  EXPECT_EQ(step_frame.cycle_index, result.output_frame.cycle_index);
}

TEST(EosSessionIntegrationTest, FusedModeDetectsInFovTarget) {
  EosSessionConfig config = MakeSessionConfig();
  config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  EosSession session = EosSessionFactory::Create(config);
  ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  input.scene.front().range_m = 1700.0f;
  input.scene.front().appearance.apparent_temperature_k = 600.0f;
  input.scene.front().appearance.projected_area_m2 = 8.0f;

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_GT(CountDetectedTargets(result.output_frame), 0U);
  EXPECT_TRUE(HasTargetId(result.output_frame, 1U));
}

TEST(EosSessionIntegrationTest, InfraredOnlyModeProducesInfraredSnr) {
  EosSessionConfig config = MakeSessionConfig();
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  EosSession session = EosSessionFactory::Create(config);
  const ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  ASSERT_FALSE(result.output_frame.detections.empty());
  const output::EosDetectionRecord& record = result.output_frame.detections.front();
  EXPECT_GT(record.infrared_snr_linear, 0.0f);
  EXPECT_NEAR(record.visible_snr_linear, 0.0f, 1e-6f);
}

TEST(EosSessionIntegrationTest, VisibleOnlyModeProducesVisibleSnr) {
  EosSessionConfig config = MakeSessionConfig();
  config.mission.work_mode = config::EosWorkMode::kVisibleOnly;
  EosSession session = EosSessionFactory::Create(config);
  const ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  ASSERT_FALSE(result.output_frame.detections.empty());
  const output::EosDetectionRecord& record = result.output_frame.detections.front();
  EXPECT_NEAR(record.infrared_snr_linear, 0.0f, 1e-6f);
  EXPECT_GT(record.visible_snr_linear, 0.0f);
}

TEST(EosSessionIntegrationTest, FusedSnrExceedsBothSingleChannelSnrInDay) {
  EosSessionConfig fused_config = MakeSessionConfig();
  fused_config.mission.work_mode = config::EosWorkMode::kFused;
  fused_config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;

  EosSessionConfig ir_config = fused_config;
  ir_config.mission.work_mode = config::EosWorkMode::kInfraredOnly;

  EosSessionConfig vis_config = fused_config;
  vis_config.mission.work_mode = config::EosWorkMode::kVisibleOnly;

  EosSession fused_session = EosSessionFactory::Create(fused_config);
  EosSession ir_session = EosSessionFactory::Create(ir_config);
  EosSession vis_session = EosSessionFactory::Create(vis_config);

  const ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();

  const session::EosOutputFrame fused_frame = fused_session.Step(input);
  const session::EosOutputFrame ir_frame = ir_session.Step(input);
  const session::EosOutputFrame vis_frame = vis_session.Step(input);

  ASSERT_FALSE(fused_frame.detections.empty());
  ASSERT_FALSE(ir_frame.detections.empty());
  ASSERT_FALSE(vis_frame.detections.empty());
  const float fused_snr = fused_frame.detections.front().fused_snr_linear;
  const float ir_snr = ir_frame.detections.front().fused_snr_linear;
  const float vis_snr = vis_frame.detections.front().fused_snr_linear;
  EXPECT_GE(fused_snr, std::min(ir_snr, vis_snr));
  EXPECT_LE(fused_snr, std::max(ir_snr, vis_snr));
}

TEST(EosSessionIntegrationTest, EmptyTargetSceneProducesEmptyDetections) {
  EosSession session = EosSessionFactory::Create(MakeSessionConfig());
  ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  input.scene.clear();

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.output_frame.detections.empty());
}

TEST(EosSessionIntegrationTest, OutOfFovTargetIsFiltered) {
  EosSession session = EosSessionFactory::Create(MakeSessionConfig());
  ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  input.scene.clear();
  input.scene.push_back(MakeTarget(10U, 50.0f, 1500.0f));

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(HasTargetId(result.output_frame, 10U));
}

TEST(EosSessionIntegrationTest, HigherCloudCoverageReducesVisibleSnr) {
  EosSessionConfig config = MakeSessionConfig();
  config.mission.work_mode = config::EosWorkMode::kVisibleOnly;
  config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;

  EosSession clear_session = EosSessionFactory::Create(config);
  EosSession cloudy_session = EosSessionFactory::Create(config);

  ::electro_optical_sensor::session::EosCycleInput clear_input = MakeBaseInput();
  clear_input.environment.cloud_coverage_ratio = 0.0f;

  ::electro_optical_sensor::session::EosCycleInput cloudy_input = MakeBaseInput();
  cloudy_input.environment.cloud_coverage_ratio = 0.9f;

  const session::EosOutputFrame clear_frame = clear_session.Step(clear_input);
  const session::EosOutputFrame cloudy_frame = cloudy_session.Step(cloudy_input);

  ASSERT_FALSE(clear_frame.detections.empty());
  ASSERT_FALSE(cloudy_frame.detections.empty());
  EXPECT_GT(clear_frame.detections.front().fused_snr_linear,
            cloudy_frame.detections.front().fused_snr_linear);
}

TEST(EosSessionIntegrationTest, WorseAtmosphereObservationReducesInfraredSnr) {
  EosSessionConfig config = MakeSessionConfig();
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;

  EosSession good_atm_session = EosSessionFactory::Create(config);
  EosSession poor_atm_session = EosSessionFactory::Create(config);

  ::electro_optical_sensor::session::EosCycleInput good_input = MakeBaseInput();
  good_input.environment.cloud_coverage_ratio = 0.1f;
  good_input.environment.ambient_wind_speed_mps = 3.0f;

  ::electro_optical_sensor::session::EosCycleInput poor_input = MakeBaseInput();
  poor_input.environment.cloud_coverage_ratio = 0.9f;
  poor_input.environment.ambient_wind_speed_mps = 70.0f;

  const session::EosOutputFrame good_frame = good_atm_session.Step(good_input);
  const session::EosOutputFrame poor_frame = poor_atm_session.Step(poor_input);

  ASSERT_FALSE(good_frame.detections.empty());
  ASSERT_FALSE(poor_frame.detections.empty());
  EXPECT_GT(good_frame.detections.front().fused_snr_linear,
            poor_frame.detections.front().fused_snr_linear);
}

TEST(EosSessionIntegrationTest, NightShiftsFusedWeightTowardInfrared) {
  EosSessionConfig fused_config = MakeSessionConfig();
  fused_config.mission.work_mode = config::EosWorkMode::kFused;
  fused_config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  EosSessionConfig ir_config = fused_config;
  ir_config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  EosSessionConfig vis_config = fused_config;
  vis_config.mission.work_mode = config::EosWorkMode::kVisibleOnly;

  EosSession day_fused = EosSessionFactory::Create(fused_config);
  EosSession day_ir = EosSessionFactory::Create(ir_config);
  EosSession day_vis = EosSessionFactory::Create(vis_config);

  ::electro_optical_sensor::session::EosCycleInput day_input = MakeBaseInput();
  day_input.environment.day_night_type = ::electro_optical_sensor::session::DayNightType::kDay;

  const session::EosOutputFrame day_fused_frame = day_fused.Step(day_input);
  const session::EosOutputFrame day_ir_frame = day_ir.Step(day_input);
  const session::EosOutputFrame day_vis_frame = day_vis.Step(day_input);
  ASSERT_FALSE(day_fused_frame.detections.empty());
  ASSERT_FALSE(day_ir_frame.detections.empty());
  ASSERT_FALSE(day_vis_frame.detections.empty());
  const float day_dist_to_vis = std::fabs(day_fused_frame.detections.front().fused_snr_linear -
                                          day_vis_frame.detections.front().fused_snr_linear);
  const float day_dist_to_ir = std::fabs(day_fused_frame.detections.front().fused_snr_linear -
                                         day_ir_frame.detections.front().fused_snr_linear);
  EXPECT_LT(day_dist_to_vis, day_dist_to_ir);

  EosSession night_fused = EosSessionFactory::Create(fused_config);
  EosSession night_ir = EosSessionFactory::Create(ir_config);
  EosSession night_vis = EosSessionFactory::Create(vis_config);

  ::electro_optical_sensor::session::EosCycleInput night_input = MakeBaseInput();
  night_input.environment.day_night_type = ::electro_optical_sensor::session::DayNightType::kNight;

  const session::EosOutputFrame night_fused_frame = night_fused.Step(night_input);
  const session::EosOutputFrame night_ir_frame = night_ir.Step(night_input);
  const session::EosOutputFrame night_vis_frame = night_vis.Step(night_input);
  ASSERT_FALSE(night_fused_frame.detections.empty());
  ASSERT_FALSE(night_ir_frame.detections.empty());
  ASSERT_FALSE(night_vis_frame.detections.empty());
  const float night_dist_to_ir = std::fabs(night_fused_frame.detections.front().fused_snr_linear -
                                           night_ir_frame.detections.front().fused_snr_linear);
  const float night_dist_to_vis = std::fabs(night_fused_frame.detections.front().fused_snr_linear -
                                            night_vis_frame.detections.front().fused_snr_linear);
  EXPECT_LT(night_dist_to_ir, night_dist_to_vis);
}

TEST(EosSessionIntegrationTest, MultiCycleScanAdvancesAzimuth) {
  EosSession session = EosSessionFactory::Create(MakeSessionConfig());

  const ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  const session::EosOutputFrame frame_1 = session.Step(input);

  ::electro_optical_sensor::session::EosCycleInput input_2 = MakeBaseInput();
  input_2.cycle_index = 2U;
  const session::EosOutputFrame frame_2 = session.Step(input_2);

  EXPECT_NE(frame_1.scan_azimuth_deg, frame_2.scan_azimuth_deg);
}

TEST(EosSessionIntegrationTest, RuntimeConfigWorkModeSwitchTakesEffectImmediately) {
  EosSession session = EosSessionFactory::Create(MakeSessionConfig());
  const ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();

  const session::EosOutputFrame fused_frame = session.Step(input);
  ASSERT_FALSE(fused_frame.detections.empty());
  EXPECT_GT(fused_frame.detections.front().infrared_snr_linear, 0.0f);
  EXPECT_GT(fused_frame.detections.front().visible_snr_linear, 0.0f);

  const EosRuntimeConfigPatch patch = eos_config::EosRuntimeConfigBuilder()
                                          .WithWorkMode(config::EosWorkMode::kInfraredOnly)
                                          .Build();
  session.ApplyRuntimeConfig(patch);

  ::electro_optical_sensor::session::EosCycleInput input_2 = MakeBaseInput();
  input_2.cycle_index = 2U;
  const session::EosOutputFrame ir_frame = session.Step(input_2);
  ASSERT_FALSE(ir_frame.detections.empty());
  EXPECT_GT(ir_frame.detections.front().infrared_snr_linear, 0.0f);
  EXPECT_NEAR(ir_frame.detections.front().visible_snr_linear, 0.0f, 1e-6f);
}

TEST(EosSessionIntegrationTest, RuntimeConfigScanRateChangeUpdatesAdvance) {
  EosSession session = EosSessionFactory::Create(MakeSessionConfig());

  const ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  const session::EosOutputFrame frame_1 = session.Step(input);

  const EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithScanRateDegPerSec(97.0f).Build();
  session.ApplyRuntimeConfig(patch);

  ::electro_optical_sensor::session::EosCycleInput input_2 = MakeBaseInput();
  input_2.cycle_index = 2U;
  const session::EosOutputFrame frame_2 = session.Step(input_2);

  const float delta_fast = std::fabs(frame_2.scan_azimuth_deg - frame_1.scan_azimuth_deg);

  EosSession slow_session = EosSessionFactory::Create(MakeSessionConfig());
  const session::EosOutputFrame slow_1 = slow_session.Step(input);
  ::electro_optical_sensor::session::EosCycleInput input_3 = MakeBaseInput();
  input_3.cycle_index = 2U;
  const session::EosOutputFrame slow_2 = slow_session.Step(input_3);
  const float delta_slow = std::fabs(slow_2.scan_azimuth_deg - slow_1.scan_azimuth_deg);

  EXPECT_GT(delta_fast, delta_slow);
}

TEST(EosSessionIntegrationTest, RuntimeConfigSnrThresholdFiltersWeakTargets) {
  EosSessionConfig config = MakeSessionConfig();
  config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  EosSession session = EosSessionFactory::Create(config);
  ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  input.scene.front().range_m = 1700.0f;
  input.scene.front().appearance.apparent_temperature_k = 600.0f;
  input.scene.front().appearance.projected_area_m2 = 8.0f;

  const session::EosOutputFrame baseline_frame = session.Step(input);
  ASSERT_GT(CountDetectedTargets(baseline_frame), 0U);

  const EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithDetectionProfile(eos_config::EosDetectionProfile::kConservative)
          .Build();
  session.ApplyRuntimeConfig(patch);

  ::electro_optical_sensor::session::EosCycleInput input_2 = input;
  input_2.cycle_index = 2U;
  const session::EosOutputFrame filtered_frame = session.Step(input_2);
  EXPECT_LE(CountDetectedTargets(filtered_frame), CountDetectedTargets(baseline_frame));
  ASSERT_FALSE(filtered_frame.detections.empty());
  ASSERT_FALSE(baseline_frame.detections.empty());
  EXPECT_LE(filtered_frame.detections.front().fused_snr_db,
            baseline_frame.detections.front().fused_snr_db);
}

TEST(EosSessionIntegrationTest, SessionConfigBuilderProducesSameResultAsDirectConfig) {
  const EosSessionConfig direct_config = MakeSessionConfig();
  const EosSessionConfig built_config =
      eos_config::EosSessionConfigBuilder(MakeSessionConfig())
          .Mission()
          .WithWorkMode(config::EosWorkMode::kFused)
          .End()
          .Detection()
          .WithDetectionProfile(eos_config::EosDetectionProfile::kAggressive)
          .End()
          .Build();

  EosSession direct_session = EosSessionFactory::Create(direct_config);
  EosSession built_session = EosSessionFactory::Create(built_config);

  const ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  const session::EosOutputFrame direct_frame = direct_session.Step(input);
  const session::EosOutputFrame built_frame = built_session.Step(input);

  EXPECT_EQ(direct_frame.detections.size(), built_frame.detections.size());
  if (!direct_frame.detections.empty() && !built_frame.detections.empty()) {
    EXPECT_FLOAT_EQ(direct_frame.detections.front().fused_snr_linear,
                    built_frame.detections.front().fused_snr_linear);
  }
}

TEST(EosSessionIntegrationTest, ValidationFailureReturnsEmptyFrameAndStillAdvancesCycleIndex) {
  EosSession session = EosSessionFactory::Create(MakeSessionConfig());
  ::electro_optical_sensor::session::EosCycleInput invalid_input = MakeBaseInput();
  invalid_input.dt_sec = 0.0f;

  const ::electro_optical_sensor::session::EosCycleResult invalid_result =
      session.StepWithResult(invalid_input);
  EXPECT_TRUE(invalid_result.has_validation_error);
  EXPECT_FALSE(invalid_result.executed_this_cycle);
  EXPECT_FALSE(invalid_result.reused_previous_output);
  EXPECT_TRUE(invalid_result.output_frame.detections.empty());
  EXPECT_EQ(invalid_result.output_frame.cycle_index, invalid_input.cycle_index);

  ::electro_optical_sensor::session::EosCycleInput valid_input = MakeBaseInput();
  valid_input.cycle_index = 2U;
  const ::electro_optical_sensor::session::EosCycleResult valid_result =
      session.StepWithResult(valid_input);
  EXPECT_FALSE(valid_result.has_validation_error);
  EXPECT_TRUE(valid_result.executed_this_cycle);
  EXPECT_FALSE(valid_result.reused_previous_output);
  EXPECT_EQ(valid_result.output_frame.cycle_index, 2U);

  ::electro_optical_sensor::session::EosCycleInput invalid_after_valid = MakeBaseInput();
  invalid_after_valid.cycle_index = 3U;
  invalid_after_valid.dt_sec = 0.0f;
  const ::electro_optical_sensor::session::EosCycleResult invalid_after_valid_result =
      session.StepWithResult(invalid_after_valid);
  EXPECT_TRUE(invalid_after_valid_result.has_validation_error);
  EXPECT_FALSE(invalid_after_valid_result.executed_this_cycle);
  EXPECT_TRUE(invalid_after_valid_result.reused_previous_output);
  EXPECT_EQ(invalid_after_valid_result.output_frame.cycle_index,
            valid_result.output_frame.cycle_index);
  EXPECT_EQ(invalid_after_valid_result.output_frame.detections.size(),
            valid_result.output_frame.detections.size());
}

TEST(EosSessionIntegrationTest, StepReusesPreviousOutputWhenValidationFailsAfterSuccessfulCycle) {
  EosSession session = EosSessionFactory::Create(MakeSessionConfig());

  ::electro_optical_sensor::session::EosCycleInput valid_input = MakeBaseInput();
  valid_input.cycle_index = 10U;
  const session::EosOutputFrame valid_frame = session.Step(valid_input);

  ::electro_optical_sensor::session::EosCycleInput invalid_input = MakeBaseInput();
  invalid_input.cycle_index = 11U;
  invalid_input.dt_sec = 0.0f;
  const session::EosOutputFrame invalid_frame = session.Step(invalid_input);

  EXPECT_EQ(invalid_frame.cycle_index, valid_frame.cycle_index);
  EXPECT_EQ(invalid_frame.detections.size(), valid_frame.detections.size());
  if (!valid_frame.detections.empty() && !invalid_frame.detections.empty()) {
    EXPECT_EQ(invalid_frame.detections.front().target_id, valid_frame.detections.front().target_id);
    EXPECT_FLOAT_EQ(invalid_frame.detections.front().fused_snr_linear,
                    valid_frame.detections.front().fused_snr_linear);
  }
}

TEST(EosSessionIntegrationTest, StraylightFilterReducesOffAxisTargetSnr) {
  EosSessionConfig no_filter_config = MakeSessionConfig();
  no_filter_config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  no_filter_config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  no_filter_config.policy.stray_light.profile = eos_config::EosStrayLightProfile::kDisabled;

  EosSessionConfig filter_config = no_filter_config;
  filter_config.policy.stray_light.profile = eos_config::EosStrayLightProfile::kEnhancedHood;

  EosSession no_filter_session = EosSessionFactory::Create(no_filter_config);
  EosSession filter_session = EosSessionFactory::Create(filter_config);

  ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  input.scene.clear();
  input.environment.solar_altitude_deg = 75.0f;
  input.environment.solar_azimuth_deg = 90.0f;
  session::EosSceneTarget target = MakeTarget(1U, 0.0f, 1000.0f, 6.0f);
  target.appearance.apparent_temperature_k = 500.0f;
  input.scene.push_back(target);

  const session::EosOutputFrame no_filter_frame = no_filter_session.Step(input);
  const session::EosOutputFrame filter_frame = filter_session.Step(input);

  ASSERT_FALSE(no_filter_frame.detections.empty());
  ASSERT_FALSE(filter_frame.detections.empty());
  EXPECT_GE(no_filter_frame.detections.front().fused_snr_linear,
            filter_frame.detections.front().fused_snr_linear);
}

TEST(EosSessionIntegrationTest, MultipleTargetsInFovAllDetected) {
  EosSession session = EosSessionFactory::Create(MakeSessionConfig());
  ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  input.scene.clear();
  input.scene.push_back(MakeTarget(101U, -3.0f, 1200.0f, 3.0f));
  input.scene.push_back(MakeTarget(102U, 0.0f, 1500.0f, 4.0f));
  input.scene.push_back(MakeTarget(103U, 4.0f, 1800.0f, 5.0f));

  const session::EosOutputFrame frame = session.Step(input);

  EXPECT_EQ(frame.detections.size(), 3U);
  EXPECT_TRUE(HasTargetId(frame, 101U));
  EXPECT_TRUE(HasTargetId(frame, 102U));
  EXPECT_TRUE(HasTargetId(frame, 103U));
}

TEST(EosSessionIntegrationTest, CloserTargetHasHigherSnrAtSameTemperature) {
  EosSessionConfig config = MakeSessionConfig();
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  EosSession session = EosSessionFactory::Create(config);

  ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  input.scene.clear();
  session::EosSceneTarget near_target = MakeTarget(201U, 0.0f, 600.0f, 4.0f);
  near_target.appearance.apparent_temperature_k = 600.0f;
  session::EosSceneTarget far_target = MakeTarget(202U, 0.0f, 3000.0f, 4.0f);
  far_target.appearance.apparent_temperature_k = 600.0f;
  input.scene.push_back(near_target);
  input.scene.push_back(far_target);

  const session::EosOutputFrame frame = session.Step(input);

  const output::EosDetectionRecord* near_rec = FindDetection(frame, 201U);
  const output::EosDetectionRecord* far_rec = FindDetection(frame, 202U);
  ASSERT_NE(near_rec, nullptr);
  ASSERT_NE(far_rec, nullptr);
  EXPECT_GT(near_rec->fused_snr_linear, far_rec->fused_snr_linear);
}

TEST(EosSessionIntegrationTest, RuntimeConfigStraylightToggleWorks) {
  EosSessionConfig config = MakeSessionConfig();
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  config.policy.stray_light.profile = eos_config::EosStrayLightProfile::kDisabled;
  EosSession session = EosSessionFactory::Create(config);

  ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  input.environment.solar_altitude_deg = 75.0f;
  input.environment.solar_azimuth_deg = 90.0f;
  input.scene.clear();
  session::EosSceneTarget target = MakeTarget(1U, 0.0f, 1000.0f, 6.0f);
  target.appearance.apparent_temperature_k = 500.0f;
  input.scene.push_back(target);

  const session::EosOutputFrame baseline_frame = session.Step(input);

  const EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithStrayLightProfile(eos_config::EosStrayLightProfile::kEnhancedHood)
          .Build();
  session.ApplyRuntimeConfig(patch);

  ::electro_optical_sensor::session::EosCycleInput input_2 = MakeBaseInput();
  input_2.cycle_index = 2U;
  input_2.environment.solar_altitude_deg = 75.0f;
  input_2.environment.solar_azimuth_deg = 90.0f;
  input_2.scene.clear();
  input_2.scene.push_back(target);

  const session::EosOutputFrame filtered_frame = session.Step(input_2);

  ASSERT_FALSE(baseline_frame.detections.empty());
  ASSERT_FALSE(filtered_frame.detections.empty());
  EXPECT_GE(baseline_frame.detections.front().fused_snr_linear,
            filtered_frame.detections.front().fused_snr_linear);
}

TEST(EosSessionIntegrationTest, RuntimeEnvironmentModelChangeTakesEffect) {
  EosSessionConfig config = MakeSessionConfig();
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  config.policy.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  EosSession session = EosSessionFactory::Create(config);

  ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  input.environment.cloud_coverage_ratio = 0.6f;
  input.environment.ambient_wind_speed_mps = 100.0f;
  input.scene.clear();
  session::EosSceneTarget target = MakeTarget(1U, 0.0f, 1000.0f, 8.0f);
  target.appearance.apparent_temperature_k = 700.0f;
  input.scene.push_back(target);

  const session::EosOutputFrame simplified_frame = session.Step(input);

  environment::EosEnvironmentScenarioConfig env_config;
  env_config.model_type = environment::EosEnvironmentModelType::kAdvanced;
  env_config.has_custom_overrides = true;
  env_config.custom_overrides.radiative_transfer_model =
      foundation::radiative_transfer::RadiativeTransferModel::kAdaptivePathRadiance;
  env_config.custom_overrides.aerosol_density_factor = 1.3f;
  env_config.custom_overrides.turbulence_factor = 1.8f;

  const EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithEnvironmentScenarioConfig(env_config).Build();
  session.ApplyRuntimeConfig(patch);

  ::electro_optical_sensor::session::EosCycleInput input_2 = MakeBaseInput();
  input_2.cycle_index = 2U;
  input_2.environment.cloud_coverage_ratio = 0.6f;
  input_2.environment.ambient_wind_speed_mps = 100.0f;
  input_2.scene.clear();
  input_2.scene.push_back(target);

  const session::EosOutputFrame advanced_frame = session.Step(input_2);

  ASSERT_FALSE(simplified_frame.detections.empty());
  ASSERT_FALSE(advanced_frame.detections.empty());
  EXPECT_GE(simplified_frame.detections.front().fused_snr_linear,
            advanced_frame.detections.front().fused_snr_linear);
}

}  // namespace
}  // namespace session
}  // namespace electro_optical_sensor
