/**
 * @file eos_session_factory_test.cpp
 * @brief 验证 EOS 会话工厂的默认装配路径契约。
 * @note 管线与环境服务已完全内部化，仅保留 Create() 路径。
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/extension/EosPipelineTypes.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electro_optical_sensor/session/EosSessionFactory.h"

namespace electro_optical_sensor {
namespace session {
namespace {

config::EosSessionConfig MakeSessionConfig() {
  config::EosSessionConfig config;
  config.mission.work_mode = config::EosWorkMode::kInfraredOnly;
  config.policy.detection.minimum_snr_db = 4.5f;
  config.policy.detection.detection_sensitivity_w = 0.8e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 700.0f;
  config.mission.scan_rate_deg_per_sec = 5.0f;
  config.mission.horizontal_fov_deg = 20.0f;
  config.mission.vertical_fov_deg = 4.0f;
  return config;
}

EosCycleInput MakeValidInput(std::uint32_t cycle_index) {
  EosCycleInput input;
  input.cycle_index = cycle_index;
  input.dt_sec = 1.0f;
  input.environment.solar_irradiance_w_m2 = 800.0f;
  input.environment.solar_altitude_deg = 45.0f;
  input.environment.cloud_coverage_ratio = 0.2f;
  input.environment.background_temperature_k = 289.0f;
  input.environment.day_night_type = ::electro_optical_sensor::session::DayNightType::kDay;
  return input;
}

TEST(EosSessionFactoryTest, CreateUsesDefaultPipelineAndProducesResult) {
  EosSession session = EosSessionFactory::Create(MakeSessionConfig());

  EosCycleInput input = MakeValidInput(10U);
  input.scene.clear();

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  EXPECT_FALSE(result.has_validation_error);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_EQ(result.output_frame.cycle_index, 10U);
  EXPECT_TRUE(result.output_frame.detections.empty());
}

}  // namespace
}  // namespace session
}  // namespace electro_optical_sensor
