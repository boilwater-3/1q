/**
 * @file eos_session_composition_root_test.cpp
 * @brief 验证 EOS 会话装配根的依赖组合与配置同步契约。
 * @note 管线与环境服务已完全内部化，不再支持外部注入。
 */

#include <gtest/gtest.h>

#include <memory>

#include "electro_optical_sensor/runtime/EosController.h"
#include "electro_optical_sensor/session/EosSessionCompositionRoot.h"
#include "electro_optical_sensor/pipeline/EosPipeline.h"

namespace electro_optical_sensor {
namespace session {
namespace internal {
namespace {

config::EosSessionConfig MakeSessionConfig() {
  config::EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kVisibleOnly;
  config.mission.scan_rate_deg_per_sec = 9.0f;
  config.mission.frame_rate_hz = 15.0f;
  config.policy.detection.minimum_snr_db = 60.0f;
  config.policy.detection.detection_sensitivity_w = 2.0e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 1000.0f;
  config.environment.scenario_config.model_type = config::EosEnvironmentModelType::kAdvanced;
  config.environment.scenario_config.preset = config::EosEnvironmentPreset::kDusty;
  return config;
}

TEST(EosSessionCompositionRootTest, ComposeDefaultBuildsOwnedGraphAndRuntimeAssembly) {
  const config::EosSessionConfig config = MakeSessionConfig();

  EosSessionComposition composition = EosSessionCompositionRoot::ComposeDefault(config);

  ASSERT_NE(composition.owned_pipeline, nullptr);
  ASSERT_NE(composition.owned_controller, nullptr);
  EXPECT_EQ(composition.internal_config.scan.work_mode, config.mission.work_mode);
  EXPECT_FLOAT_EQ(composition.internal_config.detection.minimum_snr_db, 60.0f);
}

}  // namespace
}  // namespace internal
}  // namespace session
}  // namespace electro_optical_sensor
