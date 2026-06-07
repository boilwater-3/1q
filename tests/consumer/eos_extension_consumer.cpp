/**
 * @file eos_extension_consumer.cpp
 * @brief 验证安装后 EOS 扩展接口可被外部工程实现并接入。
 *
 * 覆盖要点：
 *   - IEosEnvironmentService 自定义实现，并通过 EosSessionFactory 注入默认管线
 *   - EosController 可通过 EosSession::StepWithResult 间接访问
 *   - EosSession 构建、Step、StepWithResult、ApplyRuntimeConfig
 *   - HasValidationError、GetLastValidationIssues 字段可访问
 */

#include "1q/electro_optical_sensor/environment/EosEnvironmentTypes.h"
#include "1q/electro_optical_sensor/environment/IEosEnvironmentService.h"
#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electro_optical_sensor/session/EosSessionFactory.h"

namespace electro_optical_sensor {
namespace {

class DummyEosEnvironmentService : public environment::IEosEnvironmentService {
 public:
  environment::EosEnvironmentModelResult ResolveFactors(
      const environment::EosEnvironmentModelInputs& inputs) const override {
    (void)inputs;
    environment::EosEnvironmentModelResult result;
    result.aerosol_density_factor = 1.0f;
    result.turbulence_factor = 1.0f;
    result.path_radiance_scale_bias = 1.0f;
    return result;
  }
};

}  // namespace
}  // namespace electro_optical_sensor

int main() {
  // 1. Custom environment service with session factory
  electro_optical_sensor::DummyEosEnvironmentService environment_service;
  electro_optical_sensor::session::EosSession session =
      electro_optical_sensor::session::EosSessionFactory::CreateWithEnvironmentService(
          {}, environment_service);

  // 2. StepWithResult
  electro_optical_sensor::session::EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  input.environment.solar_irradiance_w_m2 = 800.0f;
  input.environment.background_temperature_k = 289.0f;
  input.environment.day_night_type = electro_optical_sensor::session::DayNightType::kDay;

  const electro_optical_sensor::session::EosCycleResult result = session.StepWithResult(input);
  if (!result.executed_this_cycle) {
    return 1;
  }
  (void)result.output_frame.detections.size();

  // 3. Step
  electro_optical_sensor::session::EosCycleInput input_2;
  input_2.cycle_index = 2U;
  input_2.dt_sec = 1.0f;
  input_2.environment.solar_irradiance_w_m2 = 800.0f;
  input_2.environment.background_temperature_k = 289.0f;
  input_2.environment.day_night_type = electro_optical_sensor::session::DayNightType::kDay;
  const electro_optical_sensor::session::EosOutputFrame frame = session.Step(input_2);
  (void)frame.cycle_index;

  // 4. Runtime config patch
  electro_optical_sensor::config::EosRuntimeConfigPatch patch;
  patch.has_frame_rate_hz = true;
  patch.frame_rate_hz = 15.0f;
  session.ApplyRuntimeConfig(patch);

  // 5. Validation access
  const electro_optical_sensor::session::ValidationIssueList& issues =
      result.validation_issues;
  (void)issues.size();

  // 6. EosController is accessible through the public header
  // (construction is done internally by session factory)
  electro_optical_sensor::extension::EosControllerRuntimeState controller_state;
  controller_state.owner_identity = nullptr;
  (void)controller_state;

  // 7. EosController types accessible
  electro_optical_sensor::extension::EosPipelineAbortReason abort_reason{
      electro_optical_sensor::extension::EosPipelineAbortReason::kNone};
  (void)abort_reason;

  return 0;
}
