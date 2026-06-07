/**
 * @file eos_session_factory_test.cpp
 * @brief 验证 EOS 会话工厂的外部注入与装配路径契约。
 * @note 管线已完全内部化，不再支持外部注入。仅保留 Create() 和
 *       CreateWithEnvironmentService() 路径。
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "1q/electro_optical_sensor/environment/EosEnvironmentTypes.h"
#include "1q/electro_optical_sensor/environment/IEosEnvironmentService.h"
#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/extension/EosPipelineTypes.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electro_optical_sensor/session/EosSessionFactory.h"

namespace electro_optical_sensor {
namespace session {
namespace {

class CountingEnvironmentService final : public environment::IEosEnvironmentService {
 public:
  environment::EosEnvironmentModelResult ResolveFactors(
      const environment::EosEnvironmentModelInputs& inputs) const override {
    ++resolve_count;
    last_inputs = inputs;
    environment::EosEnvironmentModelResult result;
    result.aerosol_density_factor = 1.0f + 0.2f * inputs.cloud_coverage_ratio;
    result.turbulence_factor = 1.0f + 0.01f * inputs.wind_speed_mps;
    result.path_radiance_scale_bias = 1.0f;
    return result;
  }

  mutable std::size_t resolve_count{0U};
  mutable environment::EosEnvironmentModelInputs last_inputs{};
};

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

TEST(EosSessionFactoryTest, CreateWithEnvironmentServiceUsesInjectedService) {
  CountingEnvironmentService environment_service;
  EosSession session =
      EosSessionFactory::CreateWithEnvironmentService(MakeSessionConfig(), environment_service);

  EosCycleInput input = MakeValidInput(9U);
  input.scene.clear();
  EosSceneTarget target;
  target.target_id = 7U;
  target.range_m = 1000.0f;
  target.azimuth_deg = -55.0f;
  target.elevation_deg = 0.0f;
  target.appearance.apparent_temperature_k = 500.0f;
  target.appearance.emissivity = 0.9f;
  target.appearance.reflectance = 0.2f;
  target.appearance.projected_area_m2 = 4.0f;
  input.scene.push_back(target);

  const ::electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);

  EXPECT_EQ(environment_service.resolve_count, 1U);
  EXPECT_FLOAT_EQ(environment_service.last_inputs.cloud_coverage_ratio, 0.2f);
  EXPECT_TRUE(result.executed_this_cycle);
  EXPECT_EQ(result.output_frame.cycle_index, 9U);
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
