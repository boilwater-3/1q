/**
 * @file esr_extension_consumer.cpp
 * @brief 验证安装后 ESR 扩展接口可被外部工程实现并接入控制器。
 *
 * 覆盖要点：
 *   - IInterceptPipeline 自定义实现并注入 EsrController
 *   - IEsrEnvironmentService 自定义实现并注入 EsrController
 *   - EsrController 构造、RunOnce、HasLatestOutputFrame、GetLatestOutputFrame
 *   - GetLastValidationIssues 字段可访问
 */

#include "1q/electronic_surveillance_radar/common/EsrOutputFrame.h"
#include "1q/electronic_surveillance_radar/core/context/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/core/context/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/core/controller/EsrController.h"
#include "1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h"
#include "1q/electronic_surveillance_radar/environment/IEsrEnvironmentService.h"
#include "1q/electronic_surveillance_radar/pipeline/IInterceptPipeline.h"
#include "1q/electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"

namespace electronic_surveillance_radar {
namespace {

class DummyInterceptPipeline : public pipeline::IInterceptPipeline {
 public:
  pipeline::InterceptCycleResult RunCycle(
      const core::context::EsrCycleInput& input_state,
      const environment::IEsrEnvironmentService& environment) override {
    (void)input_state;
    pipeline::InterceptCycleResult result;
    const environment::EsrEnvironmentSnapshot snapshot = environment.SampleEnvironment();
    (void)snapshot;
    return result;
  }

  void UpdateConfig(pipeline::InterceptPipelineConfig config) override { config_ = config; }

  void UpdateRuntimeConfig(pipeline::InterceptRuntimeConfig runtime_config) override {
    runtime_config_ = runtime_config;
  }

 private:
  pipeline::InterceptPipelineConfig config_{};
  pipeline::InterceptRuntimeConfig runtime_config_{};
};

class DummyEsrEnvironmentService : public environment::IEsrEnvironmentService {
 public:
  void BeginCycle(const environment::EsrEnvironmentCycleContext& cycle_context) override {
    snapshot_.cycle_index = cycle_context.cycle_index;
    snapshot_.dt_sec = cycle_context.dt_sec;
  }

  environment::EsrEnvironmentSnapshot SampleEnvironment() const override { return snapshot_; }

  void UpdateModelConfig(environment::EsrEnvironmentModelConfig config) override {
    model_config_ = config;
  }

 private:
  environment::EsrEnvironmentSnapshot snapshot_{};
  environment::EsrEnvironmentModelConfig model_config_{};
};

}  // namespace
}  // namespace electronic_surveillance_radar

int main() {
  electronic_surveillance_radar::DummyInterceptPipeline pipeline;
  electronic_surveillance_radar::DummyEsrEnvironmentService environment_service;

  electronic_surveillance_radar::core::controller::EsrController controller(pipeline,
                                                                            environment_service);

  electronic_surveillance_radar::core::context::EsrCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;

  controller.RunOnce(input);

  if (!controller.HasLatestOutputFrame()) {
    return 1;
  }

  const electronic_surveillance_radar::common::EsrOutputFrame& frame =
      controller.GetLatestOutputFrame();
  (void)frame.observation_output.observations.size();
  (void)frame.emitter_output.hypotheses.size();
  (void)frame.truth_evaluation_output.associations.size();

  const electronic_surveillance_radar::core::context::EsrValidationIssueList& issues =
      controller.GetLastValidationIssues();
  (void)issues.size();

  electronic_surveillance_radar::pipeline::IInterceptPipeline& pipeline_ref =
      controller.GetPipeline();
  electronic_surveillance_radar::environment::IEsrEnvironmentService& env_ref =
      controller.GetEnvironmentService();
  (void)pipeline_ref;
  (void)env_ref;

  electronic_surveillance_radar::core::context::EsrCycleInput input_2;
  input_2.cycle_index = 2U;
  input_2.dt_sec = 1.0f;
  controller.RunOnce(input_2);

  return 0;
}
