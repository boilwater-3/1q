// Copyright 2026. All Rights Reserved.
//
// @file rir_atmospheric_physics_test.cpp
// @brief 验证 RIR 环境域大气物理输入面：配置合同、校验负例与逐目标损耗接线。
//
// 对齐 AR 口径（DetectionExecution::ComputeTargetSpecificAtmosphericLossDb）：
// enable_physical_model=true 时驻留链路预算按每目标真实几何计算大气附加损耗；
// 默认关闭，行为零回归。

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "1q/remote_identification_radar/config/RirPolicyConfig.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/config/RirSessionConfigValidation.h"
#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirIssueCodes.h"
#include "RirCycleInputTestUtil.h"
#include "RirSqliteTestUtil.h"
#include "remote_identification_radar/runtime/RirController.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using session::RirCycleInput;
using session::RirSceneTarget;

// schema v1.1 最小有效库：单型号全四维模板（同 rir_feature_measurement_test）。
constexpr const char* kFeatureDatabaseSql = R"sql(
INSERT INTO meta VALUES
  ('schema_version','1.1'),
  ('database_id','rir-atmospheric-physics-test'),
  ('version','1.0.0'),
  ('created_utc','2026-08-21T00:00:00Z'),
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

bool HasCode(const session::RirIssueList& issues, const char* code) {
  for (const session::RirIssue& issue : issues) {
    if (issue.code == code) {
      return true;
    }
  }
  return false;
}

/// @brief 大气物理配置默认值合同（与 AR AtmosphericObservation 同源默认）：
///        默认关闭、标准海平面气象。
TEST(RirAtmosphericPhysicsTest, AtmosphericConfigDefaultsAreZeroRegression) {
  const config::RirEnvironmentConfig environment;
  EXPECT_FALSE(environment.atmospheric_physics.enable_physical_model);
  EXPECT_FLOAT_EQ(environment.atmospheric_physics.pressure_hpa, 1013.25f);
  EXPECT_FLOAT_EQ(environment.atmospheric_physics.temperature_k, 288.15f);
  EXPECT_FLOAT_EQ(environment.atmospheric_physics.relative_humidity, 0.5f);

  // 合法大气观测不产生 issue。
  config::RirSessionConfig session_config;
  session_config.environment.atmospheric_physics.enable_physical_model = true;
  const auto issues = config::ValidateRirSessionConfig(session_config);
  EXPECT_FALSE(HasCode(issues, session::codes::kInvalidEnvironmentSnapshot));
}

/// @brief 大气观测非法输入被拒绝（NaN 气压 / 湿度越界 / 非正温度）。
TEST(RirAtmosphericPhysicsTest, AtmosphericConfigRejectsInvalidObservation) {
  config::RirSessionConfig session_config;
  session_config.environment.atmospheric_physics.pressure_hpa =
      std::numeric_limits<float>::quiet_NaN();
  EXPECT_TRUE(HasCode(config::ValidateRirSessionConfig(session_config),
                      session::codes::kInvalidEnvironmentSnapshot));

  session_config = config::RirSessionConfig{};
  session_config.environment.atmospheric_physics.relative_humidity = 1.5f;
  EXPECT_TRUE(HasCode(config::ValidateRirSessionConfig(session_config),
                      session::codes::kInvalidEnvironmentSnapshot));

  session_config = config::RirSessionConfig{};
  session_config.environment.atmospheric_physics.temperature_k = 0.0f;
  EXPECT_TRUE(HasCode(config::ValidateRirSessionConfig(session_config),
                      session::codes::kInvalidEnvironmentSnapshot));
}

/** @brief 带四维真值特征的目标（出口①透出 snr_db 供差分断言）。 */
RirSceneTarget MakeFeaturedTarget() {
  RirSceneTarget target;
  target.external_target_id = 7U;
  target.target_name = "featured-target";
  target.position_x = 5000.0f;
  target.position_z = 2000.0f;
  target.velocity_x = 100.0f;
  target.rcs = 5.0f;
  for (float az = -5.0f; az <= 5.0f; az += 5.0f) {
    for (float el = 5.0f; el <= 30.0f; el += 10.0f) {
      session::RirAspectRcsSample aspect;
      aspect.aspect_az_deg = az;
      aspect.aspect_el_deg = el;
      aspect.rcs_dbsm = -3.0f;
      target.aspect_rcs_samples.push_back(aspect);
    }
  }
  for (float el = 15.0f; el <= 30.0f; el += 5.0f) {
    session::RirPolarizationRcsSample polarization;
    polarization.aspect_az_deg = 0.0f;
    polarization.aspect_el_deg = el;
    polarization.channel_1_rcs_dbsm = -3.0f;
    polarization.channel_2_rcs_dbsm = -6.0f;
    target.polarization_rcs_samples.push_back(polarization);
  }
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

RirCycleInput MakeInput(std::uint32_t cycle) {
  RirCycleInput input;
  input.input_cycle_index = cycle;
  input.dt_sec = 0.5;
  input.sim_time_sec = static_cast<float>(cycle - 1U) * 0.5f;
  SetDefaultTestPlatformEcef(&input);
  input.scene_targets.push_back(MakeFeaturedTarget());
  return input;
}

/// @brief 以给定环境配置跑一个识别周期，返回特征量测记录的 SNR（dB）。
float RunCycleSnrDb(const config::RirEnvironmentConfig* environment,
                    const std::string& database_path) {
  config::RirPolicyConfig policy;
  policy.detection.gate_mode = config::RirDetectionGateMode::kSnrFallback;
  policy.lifecycle.confirm_hits = 1U;
  policy.recognition.enabled = true;
  policy.recognition.database_path = database_path;
  runtime::RirController controller;
  config::RirHardwareConfig hardware;
  // 波束 20°/10°：单目标驻留指向下覆盖门可过，且目标仰角（≈21.8°）高于半俯仰
  // 波束宽（5°）——主瓣物理离地，杂波为零，大气差分不被地杂波污染。
  hardware.antenna.nominal_az_beamwidth_deg = 20.0f;
  hardware.antenna.nominal_el_beamwidth_deg = 10.0f;
  controller.SetHardware(hardware);
  config::RirMissionConfig mission;
  mission.work_mode = config::RirWorkMode::kIdentify;
  controller.UpdateRuntime(mission, policy);
  if (environment != nullptr) {
    controller.UpdateEnvironment(*environment);
  }

  session::RirOutputFrame frame;
  // 驻留中心对准目标视线角（(5000,0,2000) → el≈21.8°）：窄波束需主瓣照到目标；
  // 目标仰角高于半俯仰波束宽（5°）→ 主瓣离地，地杂波为零，大气差分不被污染。
  controller.RunCycle(MakeInput(1U), &frame, 9U,
                      config::RirAzimuthElevationDeg{0.0f, 21.8f});
  EXPECT_EQ(frame.feature_measurements.size(), 1U);
  return frame.feature_measurements[0].snr_db;
}

/// @brief 逐目标大气物理损耗接线：开启后 SNR 相对关闭严格下降，且目标仍可检测
///        （记录透出 = 通过 6 dB 回退门）。环境效果关闭时大气不产生任何影响。
TEST(RirAtmosphericPhysicsTest, AtmosphericLossLowersSnrWhenEnabled) {
  const std::string database_path =
      WriteTempSqlite("rir_atmospheric_physics.db",
                      std::string(kRecognitionSchemaSql) + kFeatureDatabaseSql);
  ASSERT_FALSE(database_path.empty());

  // 基线：环境效果全关（默认）。
  const float snr_env_off = RunCycleSnrDb(nullptr, database_path);

  // 植被档开启、大气物理关：传播/杂波路径生效。
  config::RirEnvironmentConfig env_without_atmosphere;
  env_without_atmosphere.vegetation_cover_profile =
      config::RirVegetationCoverProfile::kOpenGrassland;
  const float snr_env_on_atmosphere_off =
      RunCycleSnrDb(&env_without_atmosphere, database_path);

  // 植被档开启、大气物理开（湿热大气）：在植被路径之上再叠加逐目标大气损耗。
  config::RirEnvironmentConfig env_with_atmosphere = env_without_atmosphere;
  env_with_atmosphere.atmospheric_physics.enable_physical_model = true;
  env_with_atmosphere.atmospheric_physics.pressure_hpa = 950.0f;
  env_with_atmosphere.atmospheric_physics.temperature_k = 300.0f;
  env_with_atmosphere.atmospheric_physics.relative_humidity = 0.9f;
  const float snr_env_on_atmosphere_on =
      RunCycleSnrDb(&env_with_atmosphere, database_path);

  EXPECT_LT(snr_env_on_atmosphere_off, snr_env_off);
  EXPECT_LT(snr_env_on_atmosphere_on, snr_env_on_atmosphere_off);
  // 透出即检出（6 dB 回退门通过），链路不被环境效果摧毁。
  EXPECT_GE(snr_env_on_atmosphere_on, 6.0f);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
