// Copyright 2026. All Rights Reserved.
//
// @file rir_feature_measurement_test.cpp
// @brief 验证特征量测帧（双产品出口①）的生产路径与透出原则。
//
// 覆盖：记录逐字段透出（观测几何/效能上下文/平台位置）、全维无效不产生、
// 无特征库周期帧为空（透出原则，不虚构）、会话级拒绝周期产品为空。
// 字段语义与保真度边界冻结于 docs/review/rir_dual_product_stage_a_2026-08-18.md §3.1。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

#include "1q/remote_identification_radar/config/RirPolicyConfig.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"
#include "1q/remote_identification_radar/session/RirSession.h"
#include "RirCycleInputTestUtil.h"
#include "RirSqliteTestUtil.h"
#include "remote_identification_radar/runtime/RirController.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using session::RirCycleInput;
using session::RirCycleResult;
using session::RirCycleStatus;
using session::RirRecognitionFeatureDimension;
using session::RirSceneTarget;
using session::RirSession;

// schema v1.1 最小有效库：单型号全四维模板（元数据 + units 必填量纲）。
constexpr const char* kFeatureDatabaseSql = R"sql(
INSERT INTO meta VALUES
  ('schema_version','1.1'),
  ('database_id','rir-feature-measurement-test'),
  ('version','1.0.0'),
  ('created_utc','2026-08-18T00:00:00Z'),
  ('polarization_channels','H,V'),
  ('polarization_energy_reference','range_propagation_antenna_compensated');
INSERT INTO units VALUES
  ('rcs','dBsm'),('speed','m/s'),('altitude','m'),('acceleration','m/s2'),
  ('turn_radius','m'),('polarization','dB'),('range','m');
INSERT INTO categories VALUES ('BALLISTIC','弹道目标',1.0);
INSERT INTO models VALUES ('BALLISTIC_EXAMPLE_A','BALLISTIC','弹道目标示例 A',1.0);
INSERT INTO profiles VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',6.0,50.0,NULL,NULL,NULL,NULL);
INSERT INTO rcs_templates VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',-3.0,2.0,NULL,NULL,NULL);
INSERT INTO motion_templates VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',1800.0,300.0,50000.0,12000.0,12.0,6.0,6.0,0.5);
INSERT INTO polarization_templates VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',2.0,1.5,-6.0,2.0,5.0,4.0);
INSERT INTO range_profile_templates VALUES
  ('nominal','BALLISTIC_EXAMPLE_A',8.0,2.0,3.0,1.0,0.75,0.10,NULL);
)sql";

constexpr float kPi = 3.14159265358979f;

config::RirMissionConfig MakeIdentifyMission() {
  config::RirMissionConfig mission;
  mission.work_mode = config::RirWorkMode::kIdentify;
  return mission;
}

config::RirPolicyConfig MakeFallbackPolicy() {
  config::RirPolicyConfig policy;
  policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  policy.lifecycle.confirm_hits = 1U;
  return policy;
}

/** @brief 带全四维真值特征的目标：x=5000、z=2000 → look_az=0°、el=atan2(2000,5000)。 */
RirSceneTarget MakeFeaturedTarget() {
  RirSceneTarget target;
  target.external_target_id = 7U;
  target.target_name = "featured-target";
  target.position_x = 5000.0f;
  target.position_z = 2000.0f;
  target.velocity_x = 100.0f;
  target.rcs = 5.0f;
  target.range_m = std::sqrt(5000.0f * 5000.0f + 2000.0f * 2000.0f);
  // 视角网格覆盖 (az=0, el≈21.8°)，RCS 恒为 -3 dBsm。
  for (float az = -5.0f; az <= 5.0f; az += 5.0f) {
    for (float el = 5.0f; el <= 30.0f; el += 10.0f) {
      session::RirAspectRcsSample aspect;
      aspect.aspect_az_deg = az;
      aspect.aspect_el_deg = el;
      aspect.rcs_dbsm = -3.0f;
      target.aspect_rcs_samples.push_back(aspect);
    }
  }
  // 极化双通道样本（等强度通道，视线角附近）。
  for (float el = 15.0f; el <= 30.0f; el += 5.0f) {
    session::RirPolarizationRcsSample polarization;
    polarization.aspect_az_deg = 0.0f;
    polarization.aspect_el_deg = el;
    polarization.channel_1_rcs_dbsm = -3.0f;
    polarization.channel_2_rcs_dbsm = -6.0f;
    target.polarization_rcs_samples.push_back(polarization);
  }
  // 距离像散射中心：等强度三峰（对噪声门鲁棒），跨距 12 m。
  for (float offset_m = 0.0f; offset_m <= 12.0f; offset_m += 6.0f) {
    session::RirRangeRcsScatterer scatterer;
    scatterer.range_offset_m = offset_m;
    scatterer.rcs_dbsm = -3.0f;
    scatterer.channel_1_rcs_dbsm = -3.0f;
    scatterer.channel_2_rcs_dbsm = -6.0f;
    scatterer.phase_deg = 0.0f;
    scatterer.fluctuation_std_db = 0.0f;
    target.range_rcs_scatterers.push_back(scatterer);
  }
  return target;
}

RirCycleInput MakeInput(std::uint32_t cycle, const RirSceneTarget& target) {
  RirCycleInput input;
  input.input_cycle_index = cycle;
  input.dt_sec = 0.5;
  input.sim_time_sec = static_cast<float>(cycle - 1U) * 0.5f;
  SetDefaultTestPlatformEcef(&input);
  input.scene_targets.push_back(target);
  return input;
}

std::string WriteFeatureDatabase() {
  return WriteTempSqlite("rir_feature_measurement.db",
                         std::string(kRecognitionSchemaSql) + kFeatureDatabaseSql);
}

/// @brief 特征记录逐字段透出：观测几何、效能上下文、归属与库内键。
TEST(RirFeatureMeasurementTest, RecordCarriesObservationGeometryAndContext) {
  const std::string database_path = WriteFeatureDatabase();
  ASSERT_FALSE(database_path.empty());

  config::RirPolicyConfig policy = MakeFallbackPolicy();
  policy.recognition.enabled = true;
  policy.recognition.database_path = database_path;
  runtime::RirController controller;
  controller.SetHardware(config::RirHardwareConfig{});
  controller.UpdateRuntime(MakeIdentifyMission(), policy);

  session::RirOutputFrame frame;
  controller.RunCycle(MakeInput(1U, MakeFeaturedTarget()), &frame, 9U);

  ASSERT_EQ(frame.feature_measurements.size(), 1U);
  // 出口②同周期同键（双产品同源）。
  ASSERT_EQ(frame.recognition_outputs.size(), 1U);
  const session::RirFeatureMeasurementRecord& record = frame.feature_measurements[0];
  EXPECT_EQ(record.association_key, frame.recognition_outputs[0].association_key);
  EXPECT_NE(record.association_key, 0U);

  // 观测几何（雷达局部 ENU，az 自 +x 东起量）。
  const RirSceneTarget target = MakeFeaturedTarget();
  EXPECT_NEAR(record.look_az_deg, 0.0f, 1.0e-3f);
  EXPECT_NEAR(record.look_el_deg, std::atan2(2000.0f, 5000.0f) * 180.0f / kPi, 1.0e-3f);
  EXPECT_FLOAT_EQ(record.range_m, target.range_m);

  // 效能上下文：驻留与带宽来自任务/硬件缺省（0.05 s / 4.5 MHz）。
  EXPECT_FLOAT_EQ(record.dwell_sec, 0.05f);
  EXPECT_FLOAT_EQ(record.bandwidth_hz, 4.5e6f);
  EXPECT_TRUE(std::isfinite(record.snr_db));

  // 四维全有效（RCS 网格 + 已确认航迹运动 + 极化样本 + 散射中心）。
  const std::uint8_t all_dimensions =
      static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kRcs) |
      static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kMotion) |
      static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kPolarization) |
      static_cast<std::uint8_t>(RirRecognitionFeatureDimension::kRangeProfile);
  EXPECT_EQ(record.valid_feature_mask, all_dimensions);

  // 字段同值透出：RCS 均值=网格恒值 -3 dBsm（无偏真值取样，非加噪）。
  EXPECT_FLOAT_EQ(record.features.rcs.mean_dbsm, -3.0f);
  EXPECT_TRUE(record.features.rcs.valid);
  EXPECT_TRUE(record.features.motion.valid);
  EXPECT_TRUE(record.features.polarization.valid);
  EXPECT_TRUE(record.features.range_profile.valid);
  // 距离像：等强度三峰跨距 12 m。
  EXPECT_EQ(record.features.range_profile.peak_count, 3U);
  EXPECT_NEAR(record.features.range_profile.length_m, 12.0f, 0.6f);

  // 归属周期号/批号。
  EXPECT_EQ(record.cycle_index, 1U);
  EXPECT_EQ(record.batch_id, 9U);
}

/// @brief 平台 ECEF 位置在执行周期恒透出（has_platform_position=true）。
TEST(RirFeatureMeasurementTest, PlatformPositionAlwaysPresent) {
  const std::string database_path = WriteFeatureDatabase();
  ASSERT_FALSE(database_path.empty());

  config::RirPolicyConfig policy = MakeFallbackPolicy();
  policy.recognition.enabled = true;
  policy.recognition.database_path = database_path;
  runtime::RirController controller;
  controller.SetHardware(config::RirHardwareConfig{});
  controller.UpdateRuntime(MakeIdentifyMission(), policy);

  RirCycleInput with_position = MakeInput(1U, MakeFeaturedTarget());
  with_position.platform_position.x_m = 6378137.0;
  with_position.platform_position.y_m = 100.0;
  with_position.platform_position.z_m = -200.0;
  session::RirOutputFrame first;
  controller.RunCycle(with_position, &first, 9U);
  ASSERT_EQ(first.feature_measurements.size(), 1U);
  EXPECT_TRUE(first.feature_measurements[0].has_platform_position);
  EXPECT_DOUBLE_EQ(first.feature_measurements[0].platform_position.x_m, 6378137.0);
  EXPECT_DOUBLE_EQ(first.feature_measurements[0].platform_position.y_m, 100.0);
  EXPECT_DOUBLE_EQ(first.feature_measurements[0].platform_position.z_m, -200.0);

  RirSceneTarget second_target = MakeFeaturedTarget();
  second_target.external_target_id = 8U;
  session::RirOutputFrame second;
  controller.RunCycle(MakeInput(2U, second_target), &second, 10U);
  ASSERT_EQ(second.feature_measurements.size(), 1U);
  EXPECT_TRUE(second.feature_measurements[0].has_platform_position);
  EXPECT_NE(second.feature_measurements[0].platform_position.x_m, 0.0);
}

/// @brief 全维无效目标不产生记录（tentative 航迹 + 无真值特征样本）。
TEST(RirFeatureMeasurementTest, FeaturelessTargetProducesNoRecord) {
  config::RirPolicyConfig policy = MakeFallbackPolicy();
  policy.lifecycle.confirm_hits = 5U;  // 单周期 tentative → 运动维无效。
  runtime::RirController controller;
  controller.SetHardware(config::RirHardwareConfig{});
  controller.UpdateRuntime(MakeIdentifyMission(), policy);

  RirSceneTarget target;
  target.external_target_id = 7U;
  target.position_x = 5000.0f;
  target.position_z = 2000.0f;
  target.rcs = 5.0f;
  target.range_m = 5385.1648f;

  session::RirOutputFrame frame;
  controller.RunCycle(MakeInput(1U, target), &frame, 1U);

  // 无特征库 → 识别链未构建观测，特征帧为空（透出原则）。
  EXPECT_TRUE(frame.feature_measurements.empty());
}

/// @brief 会话级：特征库驱动的完整周期产出出口①；拒绝周期产品为空。
TEST(RirFeatureMeasurementTest, SessionEmitsFeatureMeasurementsAndRejectsCleanly) {
  const std::string database_path = WriteFeatureDatabase();
  ASSERT_FALSE(database_path.empty());

  config::RirSessionConfig config;
  config.mission.work_mode = config::RirWorkMode::kIdentify;
  config.policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  config.policy.lifecycle.confirm_hits = 1U;
  config.policy.recognition.enabled = true;
  config.policy.recognition.database_path = database_path;
  RirSession session = RirSession::Create(config);

  const RirCycleResult completed = session.StepWithResult(MakeInput(1U, MakeFeaturedTarget()));
  ASSERT_EQ(completed.status, RirCycleStatus::kCompleted);
  ASSERT_EQ(completed.output_frame.feature_measurements.size(), 1U);
  EXPECT_EQ(completed.output_frame.feature_measurements[0].association_key, 1U);

  // 校验拒绝周期：产品层与结果层均为空，不复用上一周期输出。
  RirCycleInput rejected = MakeInput(2U, MakeFeaturedTarget());
  rejected.dt_sec = 0.0;
  const RirCycleResult rejected_result = session.StepWithResult(rejected);
  EXPECT_EQ(rejected_result.status, RirCycleStatus::kRejectedInvalidInput);
  EXPECT_TRUE(rejected_result.output_frame.feature_measurements.empty());
  EXPECT_TRUE(rejected_result.track_attributions.empty());
}

/// @brief 质量门只挡积累不挡量测出口（冻结语义）：短驻留观测质量低于积累门限
///        （kMinimumObservationQuality=0.05）仍照常出口——透出点在质量门之前。
TEST(RirFeatureMeasurementTest, ShortDwellObservationStillEmitted) {
  const std::string database_path = WriteFeatureDatabase();
  ASSERT_FALSE(database_path.empty());

  config::RirMissionConfig mission = MakeIdentifyMission();
  mission.recognition_dwell_sec = 0.001f;  // 驻留质量因子 0.02 → 观测质量低于门限。
  config::RirPolicyConfig policy = MakeFallbackPolicy();
  policy.lifecycle.confirm_hits = 5U;  // 单周期 tentative → 运动维无效（驻留不缩放运动维）。
  policy.recognition.enabled = true;
  policy.recognition.database_path = database_path;
  runtime::RirController controller;
  controller.SetHardware(config::RirHardwareConfig{});
  controller.UpdateRuntime(mission, policy);

  // 提高检测 RCS 保证短驻留下 SNR 仍过 6 dB 特征门（检测 SNR 与特征提取共用）。
  RirSceneTarget target = MakeFeaturedTarget();
  target.rcs = 100.0f;

  session::RirOutputFrame frame;
  controller.RunCycle(MakeInput(1U, target), &frame, 1U);

  // 航迹与出口②照常；特征量测不受积累质量门影响，照常出口。
  ASSERT_EQ(frame.recognition_outputs.size(), 1U);
  ASSERT_EQ(frame.feature_measurements.size(), 1U);
  const session::RirFeatureMeasurementRecord& record = frame.feature_measurements[0];
  EXPECT_NE(record.valid_feature_mask, 0U);
  EXPECT_FLOAT_EQ(record.dwell_sec, 0.001f);
  // 自证前提：四维最大质量确实低于积累门限（否则本用例未覆盖该语义）。
  const float max_quality =
      std::max(std::max(record.features.rcs.quality, record.features.motion.quality),
               std::max(record.features.polarization.quality,
                        record.features.range_profile.quality));
  EXPECT_LT(max_quality, 0.05f);
}

/// @brief 超识别最大距离的键不产生特征记录（透出原则：观测在构建前即被距离门
///        跳过）；检测/关联/出口②/归属不受识别距离门影响。
TEST(RirFeatureMeasurementTest, RangeGatedTargetProducesNoFeatureRecord) {
  const std::string database_path = WriteFeatureDatabase();
  ASSERT_FALSE(database_path.empty());

  config::RirMissionConfig mission = MakeIdentifyMission();
  mission.max_range_m = 3000.0f;  // 目标斜距 ≈5385 m > 识别最大距离。
  config::RirPolicyConfig policy = MakeFallbackPolicy();
  policy.recognition.enabled = true;
  policy.recognition.database_path = database_path;
  runtime::RirController controller;
  controller.SetHardware(config::RirHardwareConfig{});
  controller.UpdateRuntime(mission, policy);

  session::RirOutputFrame frame;
  controller.RunCycle(MakeInput(1U, MakeFeaturedTarget()), &frame, 9U);

  ASSERT_EQ(frame.recognition_outputs.size(), 1U);
  EXPECT_EQ(controller.LatestTrackAttributions().size(), 1U);
  EXPECT_TRUE(frame.feature_measurements.empty());
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
