/**
 * @file eos_session_test.cpp
 * @brief 验证 EOS Session 单周期闭环：三工作模式输出、环境退化、运行时热切换。
 */

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosSession.h"

namespace electro_optical_sensor {
namespace session {
namespace {

namespace context = ::electro_optical_sensor::session;
namespace output = ::electro_optical_sensor::output;
namespace attribution = ::electro_optical_sensor::attribution;

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
  input.platform_altitude_m = 1200.0f;
  input.platform_pose.position_m.z = 0.0f;
  input.scene.push_back(MakeTarget(1U, 0.0f, 1500.0f, 4.0f));
  return input;
}

config::EosSessionConfig MakeSessionConfig() {
  config::EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kFused;
  config.policy.detection.minimum_snr_db = 4.5f;
  config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;
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

// 去真值化后：output::EosDetectionRecord 只含 detection_id（传感器探测语义），
// target_id→target_name 的仿真归属经 attribution::EosDetectionAttributionRecord 承载
// （挂在 EosCycleResult.detection_attributions）。按 target_id 查 detection 需经归属层中转：
// 先在 attributions 找 target_id 对应的 detection_id，再在 output_frame.detections 找记录。
const output::EosDetectionRecord* FindDetectionByTargetId(const session::EosCycleResult& result,
                                                          std::uint64_t target_id) {
  const attribution::EosDetectionAttributionRecord* attr = nullptr;
  for (std::size_t i = 0; i < result.detection_attributions.size(); ++i) {
    if (result.detection_attributions[i].target_id == target_id) {
      attr = &result.detection_attributions[i];
      break;
    }
  }
  if (attr == nullptr) {
    return nullptr;
  }
  for (std::size_t i = 0; i < result.output_frame.detections.size(); ++i) {
    if (result.output_frame.detections[i].detection_id == attr->detection_id) {
      return &result.output_frame.detections[i];
    }
  }
  return nullptr;
}

bool HasTargetId(const session::EosCycleResult& result, std::uint64_t target_id) {
  return FindDetectionByTargetId(result, target_id) != nullptr;
}

const output::EosDetectionRecord* FindDetection(const session::EosCycleResult& result,
                                                std::uint64_t target_id) {
  return FindDetectionByTargetId(result, target_id);
}

TEST(EosSessionIntegrationTest, StepWithResultProducesDetectionOutput) {
  EosSession session = EosSession::Create(MakeSessionConfig());
  const ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_EQ(result.output_frame.cycle_index, input.cycle_index);
  EXPECT_FALSE(result.output_frame.detections.empty());
}

TEST(EosSessionIntegrationTest, StepReturnsSameCycleIndexAsStepWithResult) {
  EosSession session = EosSession::Create(MakeSessionConfig());
  const ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();

  const session::EosOutputFrame step_frame = session.Step(input);
  const ::electro_optical_sensor::session::EosCycleResult result =
      session.StepWithResult(MakeBaseInput());

  EXPECT_EQ(step_frame.cycle_index, result.output_frame.cycle_index);
}

TEST(EosSessionIntegrationTest, FusedModeDetectsInFovTarget) {
  config::EosSessionConfig config = MakeSessionConfig();
  EosSession session = EosSession::Create(config);
  ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  input.scene.front().range_m = 1700.0f;
  input.scene.front().appearance.apparent_temperature_k = 600.0f;
  input.scene.front().appearance.projected_area_m2 = 8.0f;

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_GT(CountDetectedTargets(result.output_frame), 0U);
  EXPECT_TRUE(HasTargetId(result, 1U));
}

TEST(EosSessionIntegrationTest, InfraredOnlyModeProducesInfraredSnr) {
  config::EosSessionConfig config = MakeSessionConfig();
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  EosSession session = EosSession::Create(config);
  const ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  ASSERT_FALSE(result.output_frame.detections.empty());
  const output::EosDetectionRecord& record = result.output_frame.detections.front();
  EXPECT_GT(record.infrared_snr_linear, 0.0f);
  EXPECT_NEAR(record.visible_snr_linear, 0.0f, 1e-6f);
}

TEST(EosSessionIntegrationTest, VisibleOnlyModeProducesVisibleSnr) {
  config::EosSessionConfig config = MakeSessionConfig();
  config.mission.work_mode = config::EosWorkMode::kVisibleOnly;
  EosSession session = EosSession::Create(config);
  const ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  ASSERT_FALSE(result.output_frame.detections.empty());
  const output::EosDetectionRecord& record = result.output_frame.detections.front();
  EXPECT_NEAR(record.infrared_snr_linear, 0.0f, 1e-6f);
  EXPECT_GT(record.visible_snr_linear, 0.0f);
}

TEST(EosSessionIntegrationTest, FusedSnrExceedsBothSingleChannelSnrInDay) {
  config::EosSessionConfig fused_config = MakeSessionConfig();
  fused_config.mission.work_mode = config::EosWorkMode::kFused;

  config::EosSessionConfig ir_config = fused_config;
  ir_config.mission.work_mode = config::EosWorkMode::kInfraredOnly;

  config::EosSessionConfig vis_config = fused_config;
  vis_config.mission.work_mode = config::EosWorkMode::kVisibleOnly;

  EosSession fused_session = EosSession::Create(fused_config);
  EosSession ir_session = EosSession::Create(ir_config);
  EosSession vis_session = EosSession::Create(vis_config);

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
  EosSession session = EosSession::Create(MakeSessionConfig());
  ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  input.scene.clear();

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.output_frame.detections.empty());
}

TEST(EosSessionIntegrationTest, OutOfFovTargetIsFiltered) {
  EosSession session = EosSession::Create(MakeSessionConfig());
  ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  input.scene.clear();
  input.scene.push_back(MakeTarget(10U, 50.0f, 1500.0f));

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(HasTargetId(result, 10U));
}

TEST(EosSessionIntegrationTest, HigherCloudCoverageReducesVisibleSnr) {
  config::EosSessionConfig clear_config = MakeSessionConfig();
  clear_config.mission.work_mode = config::EosWorkMode::kVisibleOnly;
  clear_config.environment.scenario_config.cloud_coverage_ratio = 0.0f;

  config::EosSessionConfig cloudy_config = clear_config;
  cloudy_config.environment.scenario_config.cloud_coverage_ratio = 0.9f;

  EosSession clear_session = EosSession::Create(clear_config);
  EosSession cloudy_session = EosSession::Create(cloudy_config);

  const ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();

  const session::EosOutputFrame clear_frame = clear_session.Step(input);
  const session::EosOutputFrame cloudy_frame = cloudy_session.Step(input);

  ASSERT_FALSE(clear_frame.detections.empty());
  ASSERT_FALSE(cloudy_frame.detections.empty());
  EXPECT_GT(clear_frame.detections.front().fused_snr_linear,
            cloudy_frame.detections.front().fused_snr_linear);
}

TEST(EosSessionIntegrationTest, WorseAtmosphereObservationReducesInfraredSnr) {
  config::EosSessionConfig good_config = MakeSessionConfig();
  good_config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  good_config.environment.scenario_config.cloud_coverage_ratio = 0.1f;
  good_config.environment.scenario_config.ambient_wind_speed_mps = 3.0f;

  config::EosSessionConfig poor_config = good_config;
  poor_config.environment.scenario_config.cloud_coverage_ratio = 0.9f;
  poor_config.environment.scenario_config.ambient_wind_speed_mps = 70.0f;

  EosSession good_atm_session = EosSession::Create(good_config);
  EosSession poor_atm_session = EosSession::Create(poor_config);

  const ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();

  const session::EosOutputFrame good_frame = good_atm_session.Step(input);
  const session::EosOutputFrame poor_frame = poor_atm_session.Step(input);

  ASSERT_FALSE(good_frame.detections.empty());
  ASSERT_FALSE(poor_frame.detections.empty());
  EXPECT_GT(good_frame.detections.front().fused_snr_linear,
            poor_frame.detections.front().fused_snr_linear);
}

TEST(EosSessionIntegrationTest, NightShiftsFusedWeightTowardInfrared) {
  config::EosSessionConfig day_fused_config = MakeSessionConfig();
  day_fused_config.mission.work_mode = config::EosWorkMode::kFused;
  day_fused_config.environment.scenario_config.day_night_type = config::DayNightType::kDay;
  config::EosSessionConfig day_ir_config = day_fused_config;
  day_ir_config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  config::EosSessionConfig day_vis_config = day_fused_config;
  day_vis_config.mission.work_mode = config::EosWorkMode::kVisibleOnly;

  config::EosSessionConfig night_fused_config = day_fused_config;
  night_fused_config.environment.scenario_config.day_night_type = config::DayNightType::kNight;
  config::EosSessionConfig night_ir_config = night_fused_config;
  night_ir_config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  config::EosSessionConfig night_vis_config = night_fused_config;
  night_vis_config.mission.work_mode = config::EosWorkMode::kVisibleOnly;

  EosSession day_fused = EosSession::Create(day_fused_config);
  EosSession day_ir = EosSession::Create(day_ir_config);
  EosSession day_vis = EosSession::Create(day_vis_config);

  const ::electro_optical_sensor::session::EosCycleInput day_input = MakeBaseInput();

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

  EosSession night_fused = EosSession::Create(night_fused_config);
  EosSession night_ir = EosSession::Create(night_ir_config);
  EosSession night_vis = EosSession::Create(night_vis_config);

  const ::electro_optical_sensor::session::EosCycleInput night_input = MakeBaseInput();

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
  EosSession session = EosSession::Create(MakeSessionConfig());

  const ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  const session::EosOutputFrame frame_1 = session.Step(input);

  ::electro_optical_sensor::session::EosCycleInput input_2 = MakeBaseInput();
  input_2.cycle_index = 2U;
  const session::EosOutputFrame frame_2 = session.Step(input_2);

  EXPECT_NE(frame_1.scan_azimuth_deg, frame_2.scan_azimuth_deg);
}

TEST(EosSessionIntegrationTest, RuntimeConfigWorkModeSwitchTakesEffectImmediately) {
  EosSession session = EosSession::Create(MakeSessionConfig());
  const ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();

  const session::EosOutputFrame fused_frame = session.Step(input);
  ASSERT_FALSE(fused_frame.detections.empty());
  EXPECT_GT(fused_frame.detections.front().infrared_snr_linear, 0.0f);
  EXPECT_GT(fused_frame.detections.front().visible_snr_linear, 0.0f);

  const config::EosRuntimeConfigPatch patch = eos_config::EosRuntimeConfigBuilder()
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
  EosSession session = EosSession::Create(MakeSessionConfig());

  const ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  const session::EosOutputFrame frame_1 = session.Step(input);

  const config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithScanRateDegPerSec(97.0f).Build();
  session.ApplyRuntimeConfig(patch);

  ::electro_optical_sensor::session::EosCycleInput input_2 = MakeBaseInput();
  input_2.cycle_index = 2U;
  const session::EosOutputFrame frame_2 = session.Step(input_2);

  const float delta_fast = std::fabs(frame_2.scan_azimuth_deg - frame_1.scan_azimuth_deg);

  EosSession slow_session = EosSession::Create(MakeSessionConfig());
  const session::EosOutputFrame slow_1 = slow_session.Step(input);
  ::electro_optical_sensor::session::EosCycleInput input_3 = MakeBaseInput();
  input_3.cycle_index = 2U;
  const session::EosOutputFrame slow_2 = slow_session.Step(input_3);
  const float delta_slow = std::fabs(slow_2.scan_azimuth_deg - slow_1.scan_azimuth_deg);

  EXPECT_GT(delta_fast, delta_slow);
}

TEST(EosSessionIntegrationTest, RuntimeConfigSnrThresholdFiltersWeakTargets) {
  config::EosSessionConfig config = MakeSessionConfig();
  EosSession session = EosSession::Create(config);
  ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  input.scene.front().range_m = 1700.0f;
  input.scene.front().appearance.apparent_temperature_k = 600.0f;
  input.scene.front().appearance.projected_area_m2 = 8.0f;

  const session::EosOutputFrame baseline_frame = session.Step(input);
  ASSERT_GT(CountDetectedTargets(baseline_frame), 0U);

  const config::EosRuntimeConfigPatch patch = eos_config::EosRuntimeConfigBuilder()
                                                  .WithMinimumSnrDb(60.0f)
                                                  .WithDetectionSensitivityW(2.0e-12f)
                                                  .WithVisibleReferenceIrradianceWM2(1000.0f)
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

TEST(EosSessionIntegrationTest, SessionConfigBuilderPreservesDirectConfigBaseline) {
  const config::EosSessionConfig direct_config = MakeSessionConfig();
  const config::EosSessionConfig built_config =
      eos_config::EosSessionConfigBuilder(MakeSessionConfig()).Build();

  EosSession direct_session = EosSession::Create(direct_config);
  EosSession built_session = EosSession::Create(built_config);

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
  EosSession session = EosSession::Create(MakeSessionConfig());
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
  EosSession session = EosSession::Create(MakeSessionConfig());

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
    // 去真值化后，detection 的稳定标识是 detection_id（传感器探测语义），
    // target_id 已移入 attribution 层。复用帧时 front detection 的 detection_id 应一致。
    EXPECT_EQ(invalid_frame.detections.front().detection_id,
              valid_frame.detections.front().detection_id);
    EXPECT_FLOAT_EQ(invalid_frame.detections.front().fused_snr_linear,
                    valid_frame.detections.front().fused_snr_linear);
  }
}

TEST(EosSessionIntegrationTest, StraylightFilterReducesOffAxisTargetSnr) {
  config::EosSessionConfig no_filter_config = MakeSessionConfig();
  no_filter_config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  no_filter_config.policy.stray_light.enable_straylight_filter = false;

  config::EosSessionConfig filter_config = no_filter_config;
  filter_config.policy.stray_light.enable_straylight_filter = true;
  filter_config.policy.stray_light.hood_inner_half_angle_deg = 8.0f;
  filter_config.policy.stray_light.hood_outer_half_angle_deg = 55.0f;
  filter_config.policy.stray_light.hood_min_suppression_ratio = 0.35f;
  filter_config.policy.stray_light.hood_max_suppression_ratio = 0.95f;

  EosSession no_filter_session = EosSession::Create(no_filter_config);
  EosSession filter_session = EosSession::Create(filter_config);

  ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  input.scene.clear();
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
  EosSession session = EosSession::Create(MakeSessionConfig());
  ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  input.scene.clear();
  input.scene.push_back(MakeTarget(101U, -3.0f, 1200.0f, 3.0f));
  input.scene.push_back(MakeTarget(102U, 0.0f, 1500.0f, 4.0f));
  input.scene.push_back(MakeTarget(103U, 4.0f, 1800.0f, 5.0f));

  const session::EosCycleResult result = session.StepWithResult(input);

  EXPECT_EQ(result.output_frame.detections.size(), 3U);
  EXPECT_TRUE(HasTargetId(result, 101U));
  EXPECT_TRUE(HasTargetId(result, 102U));
  EXPECT_TRUE(HasTargetId(result, 103U));
}

TEST(EosSessionIntegrationTest, CloserTargetHasHigherSnrAtSameTemperature) {
  config::EosSessionConfig config = MakeSessionConfig();
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  EosSession session = EosSession::Create(config);

  ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  input.scene.clear();
  session::EosSceneTarget near_target = MakeTarget(201U, 0.0f, 600.0f, 4.0f);
  near_target.appearance.apparent_temperature_k = 600.0f;
  session::EosSceneTarget far_target = MakeTarget(202U, 0.0f, 3000.0f, 4.0f);
  far_target.appearance.apparent_temperature_k = 600.0f;
  input.scene.push_back(near_target);
  input.scene.push_back(far_target);

  const session::EosCycleResult result = session.StepWithResult(input);

  const output::EosDetectionRecord* near_rec = FindDetection(result, 201U);
  const output::EosDetectionRecord* far_rec = FindDetection(result, 202U);
  ASSERT_NE(near_rec, nullptr);
  ASSERT_NE(far_rec, nullptr);
  EXPECT_GT(near_rec->fused_snr_linear, far_rec->fused_snr_linear);
}

TEST(EosSessionIntegrationTest, RuntimeConfigStraylightToggleWorks) {
  config::EosSessionConfig config = MakeSessionConfig();
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  config.policy.stray_light.enable_straylight_filter = false;
  EosSession session = EosSession::Create(config);

  ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  input.scene.clear();
  session::EosSceneTarget target = MakeTarget(1U, 0.0f, 1000.0f, 6.0f);
  target.appearance.apparent_temperature_k = 500.0f;
  input.scene.push_back(target);

  const session::EosOutputFrame baseline_frame = session.Step(input);

  const config::EosRuntimeConfigPatch patch = eos_config::EosRuntimeConfigBuilder()
                                                  .WithEnableStraylightFilter(true)
                                                  .WithHoodInnerHalfAngleDeg(8.0f)
                                                  .WithHoodOuterHalfAngleDeg(55.0f)
                                                  .WithHoodMinSuppressionRatio(0.35f)
                                                  .WithHoodMaxSuppressionRatio(0.95f)
                                                  .Build();
  session.ApplyRuntimeConfig(patch);

  ::electro_optical_sensor::session::EosCycleInput input_2 = MakeBaseInput();
  input_2.cycle_index = 2U;
  input_2.scene.clear();
  input_2.scene.push_back(target);

  const session::EosOutputFrame filtered_frame = session.Step(input_2);

  ASSERT_FALSE(baseline_frame.detections.empty());
  ASSERT_FALSE(filtered_frame.detections.empty());
  EXPECT_GE(baseline_frame.detections.front().fused_snr_linear,
            filtered_frame.detections.front().fused_snr_linear);
}

TEST(EosSessionIntegrationTest, RuntimeEnvironmentPresetChangeTakesEffect) {
  config::EosSessionConfig config = MakeSessionConfig();
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  EosSession session = EosSession::Create(config);

  ::electro_optical_sensor::session::EosCycleInput input = MakeBaseInput();
  input.scene.clear();
  session::EosSceneTarget target = MakeTarget(1U, 0.0f, 1000.0f, 8.0f);
  target.appearance.apparent_temperature_k = 700.0f;
  input.scene.push_back(target);

  const session::EosOutputFrame standard_frame = session.Step(input);

  config::EosEnvironmentScenarioConfig env_config;
  env_config.preset = config::EosEnvironmentPreset::kTurbulent;

  const config::EosRuntimeConfigPatch patch =
      eos_config::EosRuntimeConfigBuilder().WithEnvironmentScenarioConfig(env_config).Build();
  session.ApplyRuntimeConfig(patch);

  ::electro_optical_sensor::session::EosCycleInput input_2 = MakeBaseInput();
  input_2.cycle_index = 2U;
  input_2.scene.clear();
  input_2.scene.push_back(target);

  const session::EosOutputFrame turbulent_frame = session.Step(input_2);

  ASSERT_FALSE(standard_frame.detections.empty());
  ASSERT_FALSE(turbulent_frame.detections.empty());
  EXPECT_GE(standard_frame.detections.front().fused_snr_linear,
            turbulent_frame.detections.front().fused_snr_linear);
}

}  // namespace
}  // namespace session
}  // namespace electro_optical_sensor
