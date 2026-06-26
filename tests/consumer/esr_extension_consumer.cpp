/**
 * @file esr_extension_consumer.cpp
 * @brief 验证安装后 ESR 扩展接口可被外部工程实现并访问。
 *
 * 覆盖要点：
 *   - IEsrEnvironmentService 自定义实现，并通过 EsrSessionFactory 注入默认管线
 *   - EsrPipelineAbortReason 公共结果类型可达
 *   - EsrSession 构建、Step、StepWithResult、ApplyRuntimeConfig
 *   - GetLastValidationIssues 字段可访问
 */

#include "1q/electronic_surveillance_radar/extension/InterceptPipelineTypes.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"
#include "1q/electronic_surveillance_radar/session/EsrSessionFactory.h"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
#include "1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h"

namespace electronic_surveillance_radar {
namespace {

class DummyEsrEnvironmentService : public environment::IEsrEnvironmentService {
 public:
  void BeginCycle(const environment::EsrEnvironmentCycleContext& cycle_context) override {
    snapshot_.cycle_index = cycle_context.cycle_index;
    snapshot_.dt_sec = cycle_context.dt_sec;
  }

  environment::EsrEnvironmentSnapshot SampleEnvironment() const override { return snapshot_; }

  void UpdateModelConfig(environment::EsrEnvironmentScenarioConfig config) override {
    model_config_ = config;
  }

 private:
  environment::EsrEnvironmentSnapshot snapshot_{};
  environment::EsrEnvironmentScenarioConfig model_config_{};
};

}  // namespace
}  // namespace electronic_surveillance_radar

int main() {
  // 1. Custom environment service with session factory
  electronic_surveillance_radar::DummyEsrEnvironmentService environment_service;
  electronic_surveillance_radar::session::EsrSession session =
      electronic_surveillance_radar::session::EsrSessionFactory::CreateWithEnvironmentService(
          {}, environment_service);

  // 2. StepWithResult
  electronic_surveillance_radar::session::EsrCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;

  const electronic_surveillance_radar::session::EsrCycleResult result = session.StepWithResult(input);
  if (!result.executed_this_cycle) {
    return 1;
  }
  (void)result.output_frame.observation_output.observations.size();
  (void)result.output_frame.emitter_output.hypotheses.size();

  // 3. Step
  electronic_surveillance_radar::session::EsrCycleInput input_2;
  input_2.cycle_index = 2U;
  input_2.dt_sec = 1.0f;
  const electronic_surveillance_radar::session::EsrOutputFrame frame = session.Step(input_2);
  (void)frame.cycle_index;

  // 4. Runtime config patch
  electronic_surveillance_radar::config::EsrRuntimeConfigPatch patch;
  patch.has_sensor_enabled = true;
  patch.sensor_enabled = true;
  session.ApplyRuntimeConfig(patch);

  // 5. Validation access
  const electronic_surveillance_radar::session::ValidationIssueList& issues =
      result.validation_issues;
  (void)issues.size();

  // 6. EsrPipelineAbortReason accessible
  electronic_surveillance_radar::extension::EsrPipelineAbortReason abort_reason{
      electronic_surveillance_radar::extension::EsrPipelineAbortReason::kNone};
  (void)abort_reason;

  return 0;
}
