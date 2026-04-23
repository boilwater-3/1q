/**
 * @file eos_extension_consumer.cpp
 * @brief 验证安装后 EOS 扩展接口可被外部工程实现并接入控制器。
 *
 * 覆盖要点：
 *   - IEosPipeline 自定义实现并注入 EosController
 *   - IEosEnvironmentService 自定义实现，并通过 EosSessionFactory 注入默认管线
 *   - EosController 构造、RunOnce、HasLatestOutputFrame、GetLatestOutputFrame
 *   - HasValidationError、GetLastValidationIssues 字段可访问
 */

#include "1q/electro_optical_sensor/output/EosOutputFrame.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "1q/electro_optical_sensor/environment/EosEnvironmentTypes.h"
#include "1q/electro_optical_sensor/extension/EosController.h"
#include "1q/electro_optical_sensor/environment/IEosEnvironmentService.h"
#include "1q/electro_optical_sensor/extension/IEosPipeline.h"

namespace electro_optical_sensor {
namespace {

class DummyEosPipeline : public extension::IEosPipeline {
 public:
  void UpdateConfig(const extension::EosPipelineConfig& config, bool reset_scan_phase) override {
    config_ = config;
    (void)reset_scan_phase;
  }

  extension::EosPipelineExecuteResult Execute(const ::electro_optical_sensor::session::EosCycleInput& input) override {
    extension::EosPipelineExecuteResult result;
    output::EosOutputFrame& frame = result.output_frame;
    frame.cycle_index = input.cycle_index;
    output::EosDetectionRecord record;
    record.target_id = 1U;
    record.range_m = 1500.0f;
    record.infrared_snr_linear = 10.0f;
    record.visible_snr_linear = 5.0f;
    record.fused_snr_linear = 12.0f;
    record.fused_snr_db = 10.79f;
    record.detected = true;
    frame.detections.push_back(record);
    result.executed_this_cycle = true;
    result.abort_reason = extension::EosPipelineAbortReason::kNone;
    return result;
  }

  extension::EosPipelineRuntimeState CaptureRuntimeState() const override {
    extension::EosPipelineRuntimeState state;
    state.owner_identity = this;
    state.schema_version = 1U;
    state.current_scan_azimuth_deg = 0.0f;
    state.scan_start_az_deg = config_.scan_start_az_deg;
    state.scan_end_az_deg = config_.scan_end_az_deg;
    state.scan_rate_deg_per_sec = config_.scan_rate_deg_per_sec;
    return state;
  }

  bool RestoreRuntimeState(const extension::EosPipelineRuntimeState& state) override {
    (void)state;
    return true;
  }

 private:
  extension::EosPipelineConfig config_{};
};

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
  electro_optical_sensor::DummyEosPipeline pipeline;

  electro_optical_sensor::extension::EosController controller(pipeline);

  electro_optical_sensor::session::EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  input.environment.solar_irradiance_w_m2 = 850.0f;
  input.environment.background_temperature_k = 289.0f;

  controller.RunOnce(input);

  if (!controller.HasLatestOutputFrame()) {
    return 1;
  }

  const electro_optical_sensor::output::EosOutputFrame& frame = controller.GetLatestOutputFrame();
  (void)frame.cycle_index;
  (void)frame.detections.size();

  if (controller.HasValidationError()) {
    return 2;
  }

  const electro_optical_sensor::session::ValidationIssueList& issues =
      controller.GetLastValidationIssues();
  (void)issues.size();

  electro_optical_sensor::extension::IEosPipeline& pipeline_ref = controller.GetPipeline();
  (void)pipeline_ref;

  electro_optical_sensor::session::EosCycleInput input_2;
  input_2.cycle_index = 2U;
  input_2.dt_sec = 1.0f;
  controller.RunOnce(input_2);

  electro_optical_sensor::DummyEosEnvironmentService environment_service;
  electro_optical_sensor::session::EosSession session =
      electro_optical_sensor::session::EosSessionFactory::CreateWithEnvironmentService(
          {}, environment_service);
  const electro_optical_sensor::output::EosOutputFrame session_frame = session.Step(input);
  (void)session_frame.cycle_index;

  return 0;
}
