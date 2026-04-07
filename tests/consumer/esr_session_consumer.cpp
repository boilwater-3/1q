/**
 * @file esr_session_consumer.cpp
 * @brief 验证安装后 ESR 公共 API 路径可被外部工程编译链接。
 *
 * 覆盖要点：
 *   - EsrSessionConfigBuilder 构造会话配置
 *   - EsrCycleInput + EmitterTruthState 构造场景输入
 *   - EsrInputValidation 输入校验
 *   - EsrSession 构造、Step、StepWithResult 调用
 *   - EsrOutputFrame 三通道输出字段可访问
 *   - EsrRuntimeConfigBuilder 热切换（传感器开关、扫描率、接收窗、检测门限）
 */

#include <cstddef>
#include <string>

#include "1q/electronic_surveillance_radar/common/EmitterTruthState.h"
#include "1q/electronic_surveillance_radar/common/EsrOutputFrame.h"
#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"
#include "1q/electronic_surveillance_radar/core/context/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/core/context/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/core/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/core/session/EsrSession.h"

namespace esr = electronic_surveillance_radar;

int main() {
  // 1. SessionConfigBuilder
  esr::core::session::EsrSessionConfig config = esr::config::EsrSessionConfigBuilder()
                                                    .WithDetectionMinSnrDb(6.0f)
                                                    .WithScanRateHz(1.0f)
                                                    .Build();

  // 2. Session construction
  esr::core::session::EsrSession session(config);

  // 3. CycleInput with a valid emitter
  esr::core::context::EsrCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;

  esr::common::EmitterTruthState emitter;
  emitter.emitter_id = "test-emitter";
  emitter.pose.position_m.x = 1000.0f;
  emitter.pose.position_m.y = 0.0f;
  emitter.pose.position_m.z = 5000.0f;
  emitter.carrier_hz = 10.0e9;
  emitter.bandwidth_hz = 2.0e6;
  emitter.tx_power_w = 1.0e6;
  emitter.pulse_width_s = 1.0e-6;
  emitter.pri_s = 1.0e-4;
  emitter.is_emitting = true;
  input.scene_emitters.push_back(emitter);

  // 4. Input validation
  const esr::core::context::EsrValidationIssueList issues =
      esr::core::context::ValidateEsrCycleInput(input);
  if (esr::core::context::HasEsrValidationError(issues)) {
    return 1;
  }

  // 5. StepWithResult
  const esr::core::session::EsrCycleResult result = session.StepWithResult(input);
  if (result.has_validation_error) {
    return 2;
  }

  // 6. Step (output-only)
  const esr::common::EsrOutputFrame step_frame = session.Step(input);

  // 7. Access three-channel output
  const std::size_t obs_count = result.output_frame.observation_output.observations.size();
  const std::size_t hyp_count = result.output_frame.emitter_output.hypotheses.size();
  const std::size_t assoc_count = result.output_frame.truth_evaluation_output.associations.size();
  const std::uint32_t obs_cycle = result.output_frame.observation_output.cycle_index;
  const std::uint32_t hyp_cycle = result.output_frame.emitter_output.cycle_index;
  const std::uint32_t eval_cycle = result.output_frame.truth_evaluation_output.cycle_index;
  (void)obs_count;
  (void)hyp_count;
  (void)assoc_count;
  (void)obs_cycle;
  (void)hyp_cycle;
  (void)eval_cycle;

  // 8. RuntimeConfigBuilder: disable sensor
  const esr::core::session::EsrRuntimeConfigPatch disable_patch =
      esr::config::EsrRuntimeConfigBuilder().WithSensorEnabled(false).Build();
  session.ApplyRuntimeConfig(disable_patch);

  // 9. Step after sensor disabled — should return empty output
  esr::core::context::EsrCycleInput input_2 = input;
  input_2.cycle_index = 2U;
  const esr::common::EsrOutputFrame disabled_frame = session.Step(input_2);
  if (!disabled_frame.observation_output.observations.empty()) {
    return 3;
  }

  // 10. RuntimeConfigBuilder: re-enable sensor
  const esr::core::session::EsrRuntimeConfigPatch enable_patch =
      esr::config::EsrRuntimeConfigBuilder().WithSensorEnabled(true).Build();
  session.ApplyRuntimeConfig(enable_patch);

  // 11. RuntimeConfigBuilder: scan rate + receive loss + detection threshold
  const esr::core::session::EsrRuntimeConfigPatch tune_patch =
      esr::config::EsrRuntimeConfigBuilder()
          .WithScanRateHz(2.0f)
          .WithIntegratedReceiveLossDb(3.0f)
          .WithDetectionMinSnrDb(12.0f)
          .Build();
  session.ApplyRuntimeConfig(tune_patch);

  // 12. RuntimeConfigBuilder: fixed receiver window
  const esr::core::session::EsrRuntimeConfigPatch window_patch =
      esr::config::EsrRuntimeConfigBuilder().WithFixedReceiverWindowHz(8.0e9, 12.0e9).Build();
  session.ApplyRuntimeConfig(window_patch);

  // 13. RuntimeConfigBuilder: disable fixed window
  const esr::core::session::EsrRuntimeConfigPatch clear_window_patch =
      esr::config::EsrRuntimeConfigBuilder().SetFixedReceiverWindowEnabled(false).Build();
  session.ApplyRuntimeConfig(clear_window_patch);

  // 14. Final cycle
  esr::core::context::EsrCycleInput input_3 = input;
  input_3.cycle_index = 3U;
  const esr::core::session::EsrCycleResult result_3 = session.StepWithResult(input_3);
  if (result_3.has_validation_error) {
    return 4;
  }

  return 0;
}
