/**
 * @file eos_session_test.cpp
 * @brief 验证 EOS Session 单周期闭环：三工作模式输出、环境退化、运行时热切换。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "1q/electro_optical_sensor/output/EosOutputFrame.h"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/model/EosCycleInput.h"
#include "1q/electro_optical_sensor/model/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electro_optical_sensor/environment/EosEnvironmentTypes.h"

namespace electro_optical_sensor {
namespace session {
namespace {

namespace context = ::electro_optical_sensor::session;
namespace model = ::electro_optical_sensor::model;
namespace output = ::electro_optical_sensor::output;

namespace eos_config = ::electro_optical_sensor::config;

session::EosTargetState MakeTarget(std::uint64_t id, float azimuth_deg, float range_m = 1800.0f,
                                   float projected_area_m2 = 2.0f) {
  session::EosTargetState target;
  target.target_id = id;
  target.range_m = range_m;
  target.azimuth_deg = azimuth_deg;
  target.elevation_deg = 0.0f;
  target.apparent_temperature_k = 330.0f;
  target.emissivity = 0.92f;
  target.reflectance = 0.38f;
  target.projected_area_m2 = projected_area_m2;
  return target;
}

session::EosCycleInput MakeBaseInput() {
  session::EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  input.solar_irradiance_w_m2 = 850.0f;
  input.solar_altitude_deg = 45.0f;
  input.atmospheric_transmittance = 0.8f;
  input.cloud_coverage_ratio = 0.2f;
  input.background_temperature_k = 289.0f;
  input.day_night_type = session::DayNightType::kDay;
  input.platform_pose.position_m.z = 1200.0f;
  input.scene_targets.push_back(MakeTarget(1U, 0.0f, 1500.0f, 4.0f));
  return input;
}

EosSessionConfig MakeSessionConfig() {
  EosSessionConfig config;
  config.scan.work_mode = EosWorkMode::kFused;
  config.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  config.pointing.scan_start_az_deg = -10.0f;
  config.pointing.scan_end_az_deg = 10.0f;
  config.scan.scan_rate_deg_per_sec = 5.0f;
  config.scan.horizontal_fov_deg = 20.0f;
  config.scan.vertical_fov_deg = 4.0f;
  return config;
}

std::size_t CountDetectedTargets(const output::EosOutputFrame& frame) {
  std::size_t count = 0U;
  for (std::size_t i = 0; i < frame.detections.size(); ++i) {
    if (frame.detections[i].detected) {
      ++count;
    }
  }
  return count;
}

bool HasTargetId(const output::EosOutputFrame& frame, std::uint64_t target_id) {
  for (std::size_t i = 0; i < frame.detections.size(); ++i) {
    if (frame.detections[i].target_id == target_id) {
      return true;
    }
  }
  return false;
}

const output::EosDetectionRecord* FindDetection(const output::EosOutputFrame& frame,
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
  const session::EosCycleInput input = MakeBaseInput();

  const ::electro_optical_sensor::model::EosCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_EQ(result.output_frame.cycle_index, input.cycle_index);
  EXPECT_FALSE(result.output_frame.detections.empty());
}

TEST(EosSessionIntegrationTest, StepReturnsSameCycleIndexAsStepWithResult) {
  EosSession session = EosSessionFactory::Create(MakeSessionConfig());
  const session::EosCycleInput input = MakeBaseInput();

  const output::EosOutputFrame step_frame = session.Step(input);
  const ::electro_optical_sensor::model::EosCycleResult result = session.StepWithResult(MakeBaseInput());

  EXPECT_EQ(step_frame.cycle_index, result.output_frame.cycle_index);
}

TEST(EosSessionIntegrationTest, FusedModeDetectsInFovTarget) {
  EosSessionConfig config = MakeSessionConfig();
  config.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  EosSession session = EosSessionFactory::Create(config);
  session::EosCycleInput input = MakeBaseInput();
  input.scene_targets.front().range_m = 1700.0f;
  input.scene_targets.front().apparent_temperature_k = 600.0f;
  input.scene_targets.front().projected_area_m2 = 8.0f;

  const ::electro_optical_sensor::model::EosCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_GT(CountDetectedTargets(result.output_frame), 0U);
  EXPECT_TRUE(HasTargetId(result.output_frame, 1U));
}

TEST(EosSessionIntegrationTest, InfraredOnlyModeProducesInfraredSnr) {
  EosSessionConfig config = MakeSessionConfig();
  config.scan.work_mode = EosWorkMode::kInfraredOnly;
  EosSession session = EosSessionFactory::Create(config);
  const session::EosCycleInput input = MakeBaseInput();

  const ::electro_optical_sensor::model::EosCycleResult result = session.StepWithResult(input);

  ASSERT_FALSE(result.output_frame.detections.empty());
  const output::EosDetectionRecord& record = result.output_frame.detections.front();
  EXPECT_GT(record.infrared_snr_linear, 0.0f);
  EXPECT_NEAR(record.visible_snr_linear, 0.0f, 1e-6f);
}

TEST(EosSessionIntegrationTest, VisibleOnlyModeProducesVisibleSnr) {
  EosSessionConfig config = MakeSessionConfig();
  config.scan.work_mode = EosWorkMode::kVisibleOnly;
  EosSession session = EosSessionFactory::Create(config);
  const session::EosCycleInput input = MakeBaseInput();

  const ::electro_optical_sensor::model::EosCycleResult result = session.StepWithResult(input);

  ASSERT_FALSE(result.output_frame.detections.empty());
  const output::EosDetectionRecord& record = result.output_frame.detections.front();
  EXPECT_NEAR(record.infrared_snr_linear, 0.0f, 1e-6f);
  EXPECT_GT(record.visible_snr_linear, 0.0f);
}

TEST(EosSessionIntegrationTest, FusedSnrExceedsBothSingleChannelSnrInDay) {
  EosSessionConfig fused_config = MakeSessionConfig();
  fused_config.scan.work_mode = EosWorkMode::kFused;
  fused_config.detection.profile = eos_config::EosDetectionProfile::kAggressive;

  EosSessionConfig ir_config = fused_config;
  ir_config.scan.work_mode = EosWorkMode::kInfraredOnly;

  EosSessionConfig vis_config = fused_config;
  vis_config.scan.work_mode = EosWorkMode::kVisibleOnly;

  EosSession fused_session = EosSessionFactory::Create(fused_config);
  EosSession ir_session = EosSessionFactory::Create(ir_config);
  EosSession vis_session = EosSessionFactory::Create(vis_config);

  const session::EosCycleInput input = MakeBaseInput();

  const output::EosOutputFrame fused_frame = fused_session.Step(input);
  const output::EosOutputFrame ir_frame = ir_session.Step(input);
  const output::EosOutputFrame vis_frame = vis_session.Step(input);

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
  session::EosCycleInput input = MakeBaseInput();
  input.scene_targets.clear();

  const ::electro_optical_sensor::model::EosCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.output_frame.detections.empty());
}

TEST(EosSessionIntegrationTest, OutOfFovTargetIsFiltered) {
  EosSession session = EosSessionFactory::Create(MakeSessionConfig());
  session::EosCycleInput input = MakeBaseInput();
  input.scene_targets.clear();
  input.scene_targets.push_back(MakeTarget(10U, 50.0f, 1500.0f));

  const ::electro_optical_sensor::model::EosCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(HasTargetId(result.output_frame, 10U));
}

TEST(EosSessionIntegrationTest, HigherCloudCoverageReducesVisibleSnr) {
  EosSessionConfig config = MakeSessionConfig();
  config.scan.work_mode = EosWorkMode::kVisibleOnly;
  config.detection.profile = eos_config::EosDetectionProfile::kAggressive;

  EosSession clear_session = EosSessionFactory::Create(config);
  EosSession cloudy_session = EosSessionFactory::Create(config);

  session::EosCycleInput clear_input = MakeBaseInput();
  clear_input.cloud_coverage_ratio = 0.0f;

  session::EosCycleInput cloudy_input = MakeBaseInput();
  cloudy_input.cloud_coverage_ratio = 0.9f;

  const output::EosOutputFrame clear_frame = clear_session.Step(clear_input);
  const output::EosOutputFrame cloudy_frame = cloudy_session.Step(cloudy_input);

  ASSERT_FALSE(clear_frame.detections.empty());
  ASSERT_FALSE(cloudy_frame.detections.empty());
  EXPECT_GT(clear_frame.detections.front().fused_snr_linear,
            cloudy_frame.detections.front().fused_snr_linear);
}

TEST(EosSessionIntegrationTest, LowerTransmittanceReducesInfraredSnr) {
  EosSessionConfig config = MakeSessionConfig();
  config.scan.work_mode = EosWorkMode::kInfraredOnly;
  config.detection.profile = eos_config::EosDetectionProfile::kAggressive;

  EosSession good_atm_session = EosSessionFactory::Create(config);
  EosSession poor_atm_session = EosSessionFactory::Create(config);

  session::EosCycleInput good_input = MakeBaseInput();
  good_input.atmospheric_transmittance = 0.95f;

  session::EosCycleInput poor_input = MakeBaseInput();
  poor_input.atmospheric_transmittance = 0.3f;

  const output::EosOutputFrame good_frame = good_atm_session.Step(good_input);
  const output::EosOutputFrame poor_frame = poor_atm_session.Step(poor_input);

  ASSERT_FALSE(good_frame.detections.empty());
  ASSERT_FALSE(poor_frame.detections.empty());
  EXPECT_GT(good_frame.detections.front().fused_snr_linear,
            poor_frame.detections.front().fused_snr_linear);
}

TEST(EosSessionIntegrationTest, NightShiftsFusedWeightTowardInfrared) {
  EosSessionConfig fused_config = MakeSessionConfig();
  fused_config.scan.work_mode = EosWorkMode::kFused;
  fused_config.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  EosSessionConfig ir_config = fused_config;
  ir_config.scan.work_mode = EosWorkMode::kInfraredOnly;
  EosSessionConfig vis_config = fused_config;
  vis_config.scan.work_mode = EosWorkMode::kVisibleOnly;

  EosSession day_fused = EosSessionFactory::Create(fused_config);
  EosSession day_ir = EosSessionFactory::Create(ir_config);
  EosSession day_vis = EosSessionFactory::Create(vis_config);

  session::EosCycleInput day_input = MakeBaseInput();
  day_input.day_night_type = session::DayNightType::kDay;

  const output::EosOutputFrame day_fused_frame = day_fused.Step(day_input);
  const output::EosOutputFrame day_ir_frame = day_ir.Step(day_input);
  const output::EosOutputFrame day_vis_frame = day_vis.Step(day_input);
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

  session::EosCycleInput night_input = MakeBaseInput();
  night_input.day_night_type = session::DayNightType::kNight;

  const output::EosOutputFrame night_fused_frame = night_fused.Step(night_input);
  const output::EosOutputFrame night_ir_frame = night_ir.Step(night_input);
  const output::EosOutputFrame night_vis_frame = night_vis.Step(night_input);
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

  const session::EosCycleInput input = MakeBaseInput();
  const output::EosOutputFrame frame_1 = session.Step(input);

  session::EosCycleInput input_2 = MakeBaseInput();
  input_2.cycle_index = 2U;
  const output::EosOutputFrame frame_2 = session.Step(input_2);

  EXPECT_NE(frame_1.scan_azimuth_deg, frame_2.scan_azimuth_deg);
}

TEST(EosSessionIntegrationTest, RuntimeConfigWorkModeSwitchTakesEffectImmediately) {
  EosSession session = EosSessionFactory::Create(MakeSessionConfig());
  const session::EosCycleInput input = MakeBaseInput();

  const output::EosOutputFrame fused_frame = session.Step(input);
  ASSERT_FALSE(fused_frame.detections.empty());
  EXPECT_GT(fused_frame.detections.front().infrared_snr_linear, 0.0f);
  EXPECT_GT(fused_frame.detections.front().visible_snr_linear, 0.0f);

  const EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithWorkMode(EosWorkMode::kInfraredOnly).Build();
  session.ApplyRuntimeConfig(patch);

  session::EosCycleInput input_2 = MakeBaseInput();
  input_2.cycle_index = 2U;
  const output::EosOutputFrame ir_frame = session.Step(input_2);
  ASSERT_FALSE(ir_frame.detections.empty());
  EXPECT_GT(ir_frame.detections.front().infrared_snr_linear, 0.0f);
  EXPECT_NEAR(ir_frame.detections.front().visible_snr_linear, 0.0f, 1e-6f);
}

TEST(EosSessionIntegrationTest, RuntimeConfigScanRateChangeUpdatesAdvance) {
  EosSession session = EosSessionFactory::Create(MakeSessionConfig());

  const session::EosCycleInput input = MakeBaseInput();
  const output::EosOutputFrame frame_1 = session.Step(input);

  const EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithScanRateDegPerSec(97.0f).Build();
  session.ApplyRuntimeConfig(patch);

  session::EosCycleInput input_2 = MakeBaseInput();
  input_2.cycle_index = 2U;
  const output::EosOutputFrame frame_2 = session.Step(input_2);

  const float delta_fast = std::fabs(frame_2.scan_azimuth_deg - frame_1.scan_azimuth_deg);

  EosSession slow_session = EosSessionFactory::Create(MakeSessionConfig());
  const output::EosOutputFrame slow_1 = slow_session.Step(input);
  session::EosCycleInput input_3 = MakeBaseInput();
  input_3.cycle_index = 2U;
  const output::EosOutputFrame slow_2 = slow_session.Step(input_3);
  const float delta_slow = std::fabs(slow_2.scan_azimuth_deg - slow_1.scan_azimuth_deg);

  EXPECT_GT(delta_fast, delta_slow);
}

TEST(EosSessionIntegrationTest, RuntimeConfigSnrThresholdFiltersWeakTargets) {
  EosSessionConfig config = MakeSessionConfig();
  config.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  EosSession session = EosSessionFactory::Create(config);
  session::EosCycleInput input = MakeBaseInput();
  input.scene_targets.front().range_m = 1700.0f;
  input.scene_targets.front().apparent_temperature_k = 600.0f;
  input.scene_targets.front().projected_area_m2 = 8.0f;

  const output::EosOutputFrame baseline_frame = session.Step(input);
  ASSERT_GT(CountDetectedTargets(baseline_frame), 0U);

  const EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithDetectionProfile(eos_config::EosDetectionProfile::kConservative)
          .Build();
  session.ApplyRuntimeConfig(patch);

  session::EosCycleInput input_2 = input;
  input_2.cycle_index = 2U;
  const output::EosOutputFrame filtered_frame = session.Step(input_2);
  EXPECT_EQ(CountDetectedTargets(filtered_frame), 0U);
}

TEST(EosSessionIntegrationTest, SessionConfigBuilderProducesSameResultAsDirectConfig) {
  const EosSessionConfig direct_config = MakeSessionConfig();
  const EosSessionConfig built_config = eos_config::EosSessionConfigBuilder(MakeSessionConfig())
                                            .WithWorkMode(EosWorkMode::kFused)
                                            .WithDetectionProfile(
                                                eos_config::EosDetectionProfile::kAggressive)
                                            .WithScanRateDegPerSec(5.0f)
                                            .Build();

  EosSession direct_session = EosSessionFactory::Create(direct_config);
  EosSession built_session = EosSessionFactory::Create(built_config);

  const session::EosCycleInput input = MakeBaseInput();
  const output::EosOutputFrame direct_frame = direct_session.Step(input);
  const output::EosOutputFrame built_frame = built_session.Step(input);

  EXPECT_EQ(direct_frame.detections.size(), built_frame.detections.size());
  if (!direct_frame.detections.empty() && !built_frame.detections.empty()) {
    EXPECT_FLOAT_EQ(direct_frame.detections.front().fused_snr_linear,
                    built_frame.detections.front().fused_snr_linear);
  }
}

TEST(EosSessionIntegrationTest, ValidationFailureReturnsEmptyFrameAndStillAdvancesCycleIndex) {
  EosSession session = EosSessionFactory::Create(MakeSessionConfig());
  session::EosCycleInput invalid_input = MakeBaseInput();
  invalid_input.dt_sec = 0.0f;

  const ::electro_optical_sensor::model::EosCycleResult invalid_result = session.StepWithResult(invalid_input);
  EXPECT_TRUE(invalid_result.has_validation_error);
  EXPECT_FALSE(invalid_result.executed_this_cycle);
  EXPECT_FALSE(invalid_result.reused_previous_output);
  EXPECT_TRUE(invalid_result.output_frame.detections.empty());
  EXPECT_EQ(invalid_result.output_frame.cycle_index, invalid_input.cycle_index);

  session::EosCycleInput valid_input = MakeBaseInput();
  valid_input.cycle_index = 2U;
  const ::electro_optical_sensor::model::EosCycleResult valid_result = session.StepWithResult(valid_input);
  EXPECT_FALSE(valid_result.has_validation_error);
  EXPECT_TRUE(valid_result.executed_this_cycle);
  EXPECT_FALSE(valid_result.reused_previous_output);
  EXPECT_EQ(valid_result.output_frame.cycle_index, 2U);

  session::EosCycleInput invalid_after_valid = MakeBaseInput();
  invalid_after_valid.cycle_index = 3U;
  invalid_after_valid.dt_sec = 0.0f;
  const ::electro_optical_sensor::model::EosCycleResult invalid_after_valid_result = session.StepWithResult(invalid_after_valid);
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

  session::EosCycleInput valid_input = MakeBaseInput();
  valid_input.cycle_index = 10U;
  const output::EosOutputFrame valid_frame = session.Step(valid_input);

  session::EosCycleInput invalid_input = MakeBaseInput();
  invalid_input.cycle_index = 11U;
  invalid_input.dt_sec = 0.0f;
  const output::EosOutputFrame invalid_frame = session.Step(invalid_input);

  EXPECT_EQ(invalid_frame.cycle_index, valid_frame.cycle_index);
  EXPECT_EQ(invalid_frame.detections.size(), valid_frame.detections.size());
  if (!valid_frame.detections.empty() && !invalid_frame.detections.empty()) {
    EXPECT_EQ(invalid_frame.detections.front().target_id,
              valid_frame.detections.front().target_id);
    EXPECT_FLOAT_EQ(invalid_frame.detections.front().fused_snr_linear,
                    valid_frame.detections.front().fused_snr_linear);
  }
}

TEST(EosSessionIntegrationTest, StraylightFilterReducesOffAxisTargetSnr) {
  EosSessionConfig no_filter_config = MakeSessionConfig();
  no_filter_config.scan.work_mode = EosWorkMode::kInfraredOnly;
  no_filter_config.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  no_filter_config.stray_light.profile = eos_config::EosStrayLightProfile::kDisabled;

  EosSessionConfig filter_config = no_filter_config;
  filter_config.stray_light.profile = eos_config::EosStrayLightProfile::kEnhancedHood;
        
  EosSession no_filter_session = EosSessionFactory::Create(no_filter_config);
  EosSession filter_session = EosSessionFactory::Create(filter_config);

  session::EosCycleInput input = MakeBaseInput();
  input.scene_targets.clear();
  input.solar_altitude_deg = 75.0f;
  input.solar_azimuth_deg = 90.0f;
  session::EosTargetState target = MakeTarget(1U, 0.0f, 1000.0f, 6.0f);
  target.apparent_temperature_k = 500.0f;
  input.scene_targets.push_back(target);

  const output::EosOutputFrame no_filter_frame = no_filter_session.Step(input);
  const output::EosOutputFrame filter_frame = filter_session.Step(input);

  ASSERT_FALSE(no_filter_frame.detections.empty());
  ASSERT_FALSE(filter_frame.detections.empty());
  EXPECT_GE(no_filter_frame.detections.front().fused_snr_linear,
            filter_frame.detections.front().fused_snr_linear);
}

TEST(EosSessionIntegrationTest, MultipleTargetsInFovAllDetected) {
  EosSession session = EosSessionFactory::Create(MakeSessionConfig());
  session::EosCycleInput input = MakeBaseInput();
  input.scene_targets.clear();
  input.scene_targets.push_back(MakeTarget(101U, -3.0f, 1200.0f, 3.0f));
  input.scene_targets.push_back(MakeTarget(102U, 0.0f, 1500.0f, 4.0f));
  input.scene_targets.push_back(MakeTarget(103U, 4.0f, 1800.0f, 5.0f));

  const output::EosOutputFrame frame = session.Step(input);

  EXPECT_EQ(frame.detections.size(), 3U);
  EXPECT_TRUE(HasTargetId(frame, 101U));
  EXPECT_TRUE(HasTargetId(frame, 102U));
  EXPECT_TRUE(HasTargetId(frame, 103U));
}

TEST(EosSessionIntegrationTest, CloserTargetHasHigherSnrAtSameTemperature) {
  EosSessionConfig config = MakeSessionConfig();
  config.scan.work_mode = EosWorkMode::kInfraredOnly;
  config.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  EosSession session = EosSessionFactory::Create(config);

  session::EosCycleInput input = MakeBaseInput();
  input.scene_targets.clear();
  session::EosTargetState near_target = MakeTarget(201U, 0.0f, 600.0f, 4.0f);
  near_target.apparent_temperature_k = 600.0f;
  session::EosTargetState far_target = MakeTarget(202U, 0.0f, 3000.0f, 4.0f);
  far_target.apparent_temperature_k = 600.0f;
  input.scene_targets.push_back(near_target);
  input.scene_targets.push_back(far_target);

  const output::EosOutputFrame frame = session.Step(input);

  const output::EosDetectionRecord* near_rec = FindDetection(frame, 201U);
  const output::EosDetectionRecord* far_rec = FindDetection(frame, 202U);
  ASSERT_NE(near_rec, nullptr);
  ASSERT_NE(far_rec, nullptr);
  EXPECT_GT(near_rec->fused_snr_linear, far_rec->fused_snr_linear);
}

TEST(EosSessionIntegrationTest, RuntimeConfigStraylightToggleWorks) {
  EosSessionConfig config = MakeSessionConfig();
  config.scan.work_mode = EosWorkMode::kInfraredOnly;
  config.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  config.stray_light.profile = eos_config::EosStrayLightProfile::kDisabled;
  EosSession session = EosSessionFactory::Create(config);

  session::EosCycleInput input = MakeBaseInput();
  input.solar_altitude_deg = 75.0f;
  input.solar_azimuth_deg = 90.0f;
  input.scene_targets.clear();
  session::EosTargetState target = MakeTarget(1U, 0.0f, 1000.0f, 6.0f);
  target.apparent_temperature_k = 500.0f;
  input.scene_targets.push_back(target);

  const output::EosOutputFrame baseline_frame = session.Step(input);

  const EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithStrayLightProfile(eos_config::EosStrayLightProfile::kEnhancedHood)
          .Build();
  session.ApplyRuntimeConfig(patch);

  session::EosCycleInput input_2 = MakeBaseInput();
  input_2.cycle_index = 2U;
  input_2.solar_altitude_deg = 75.0f;
  input_2.solar_azimuth_deg = 90.0f;
  input_2.scene_targets.clear();
  input_2.scene_targets.push_back(target);

  const output::EosOutputFrame filtered_frame = session.Step(input_2);

  ASSERT_FALSE(baseline_frame.detections.empty());
  ASSERT_FALSE(filtered_frame.detections.empty());
  EXPECT_GE(baseline_frame.detections.front().fused_snr_linear,
            filtered_frame.detections.front().fused_snr_linear);
}

TEST(EosSessionIntegrationTest, RuntimeEnvironmentModelChangeTakesEffect) {
  EosSessionConfig config = MakeSessionConfig();
  config.scan.work_mode = EosWorkMode::kInfraredOnly;
  config.detection.profile = eos_config::EosDetectionProfile::kAggressive;
  EosSession session = EosSessionFactory::Create(config);

  session::EosCycleInput input = MakeBaseInput();
  input.cloud_coverage_ratio = 0.6f;
  input.ambient_wind_speed_mps = 100.0f;
  input.scene_targets.clear();
  session::EosTargetState target = MakeTarget(1U, 0.0f, 1000.0f, 8.0f);
  target.apparent_temperature_k = 700.0f;
  input.scene_targets.push_back(target);

  const output::EosOutputFrame simplified_frame = session.Step(input);

  const EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder()
          .WithEnvironmentModelType(environment::EosEnvironmentModelType::kAdvanced)
          .WithEnvironmentPreset(eos_config::EosEnvironmentPreset::kTurbulent)
          .Build();
  session.ApplyRuntimeConfig(patch);

  session::EosCycleInput input_2 = MakeBaseInput();
  input_2.cycle_index = 2U;
  input_2.cloud_coverage_ratio = 0.6f;
  input_2.ambient_wind_speed_mps = 100.0f;
  input_2.scene_targets.clear();
  input_2.scene_targets.push_back(target);

  const output::EosOutputFrame advanced_frame = session.Step(input_2);

  ASSERT_FALSE(simplified_frame.detections.empty());
  ASSERT_FALSE(advanced_frame.detections.empty());
  EXPECT_GE(simplified_frame.detections.front().fused_snr_linear,
            advanced_frame.detections.front().fused_snr_linear);
}

}  // namespace
}  // namespace session
}  // namespace electro_optical_sensor
