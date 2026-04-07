/**
 * @file eos_extension_consumer.cpp
 * @brief 验证安装后 EOS 扩展接口可被外部工程实现并接入控制器。
 *
 * 覆盖要点：
 *   - IEosPipeline 自定义实现并注入 EosController
 *   - IEosEnvironmentService 自定义实现（EosController 仅需 pipeline）
 *   - EosController 构造、RunOnce、HasLatestOutputFrame、GetLatestOutputFrame
 *   - HasValidationError、GetLastValidationIssues 字段可访问
 */

#include "1q/electro_optical_sensor/common/EosOutputFrame.h"
#include "1q/electro_optical_sensor/core/context/EosCycleInput.h"
#include "1q/electro_optical_sensor/core/context/EosInputValidation.h"
#include "1q/electro_optical_sensor/core/controller/EosController.h"
#include "1q/electro_optical_sensor/environment/EosEnvironmentTypes.h"
#include "1q/electro_optical_sensor/environment/IEosEnvironmentService.h"
#include "1q/electro_optical_sensor/pipeline/EosPipelineTypes.h"
#include "1q/electro_optical_sensor/pipeline/IEosPipeline.h"

namespace electro_optical_sensor {
namespace {

class DummyEosPipeline : public pipeline::IEosPipeline {
 public:
  void UpdateConfig(const pipeline::EosPipelineConfig& config, bool reset_scan_phase) override {
    config_ = config;
    (void)reset_scan_phase;
  }

  common::EosOutputFrame Execute(const core::context::EosCycleInput& input) override {
    common::EosOutputFrame frame;
    frame.cycle_index = input.cycle_index;
    common::EosDetectionRecord record;
    record.target_id = 1U;
    record.range_m = 1500.0f;
    record.infrared_snr_linear = 10.0f;
    record.visible_snr_linear = 5.0f;
    record.fused_snr_linear = 12.0f;
    record.fused_snr_db = 10.79f;
    record.detected = true;
    frame.detections.push_back(record);
    return frame;
  }

 private:
  pipeline::EosPipelineConfig config_{};
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

  electro_optical_sensor::core::controller::EosController controller(pipeline);

  electro_optical_sensor::core::context::EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  input.solar_irradiance_w_m2 = 850.0f;
  input.atmospheric_transmittance = 0.8f;
  input.background_temperature_k = 289.0f;

  controller.RunOnce(input);

  if (!controller.HasLatestOutputFrame()) {
    return 1;
  }

  const electro_optical_sensor::common::EosOutputFrame& frame = controller.GetLatestOutputFrame();
  (void)frame.cycle_index;
  (void)frame.detections.size();

  if (controller.HasValidationError()) {
    return 2;
  }

  const electro_optical_sensor::core::context::EosValidationIssueList& issues =
      controller.GetLastValidationIssues();
  (void)issues.size();

  electro_optical_sensor::pipeline::IEosPipeline& pipeline_ref = controller.GetPipeline();
  (void)pipeline_ref;

  electro_optical_sensor::core::context::EosCycleInput input_2;
  input_2.cycle_index = 2U;
  input_2.dt_sec = 1.0f;
  controller.RunOnce(input_2);

  return 0;
}
