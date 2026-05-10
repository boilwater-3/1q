/**
 * @file esr_session_consumer.cpp
 * @brief 验证安装后 ESR 公共 API 路径可被外部工程编译链接。
 *
 * 覆盖要点：
 *   - EsrSessionConfigBuilder 构造会话配置
 *   - 直接字段赋值构造详细会话配置
 *   - EsrCycleInput + EsrSceneEmitter 构造场景输入
 *   - EsrInputValidation 输入校验
 *   - EsrSession 构造、Step、StepWithResult 调用
 *   - EsrOutputFrame 三通道输出字段可访问
 *   - EsrRuntimeConfigBuilder 热切换（传感器开关、扫描率、接收窗、检测门限）
 */

#include <cstddef>
#include <string>

#include "1q/electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h"
#include "1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrSession.h"

namespace esr = electronic_surveillance_radar;

int main() {
  // 1. SessionConfigBuilder
  esr::session::EsrSessionConfig config =
      esr::config::EsrSessionConfigBuilder()
          .Detection()
          .WithDetectionProfile(esr::config::EsrDetectionProfile::kBalanced)
          .End()
          .Mission()
          .WithScanRateHz(1.0f)
          .End()
          .Build();

  // 2. 直接字段赋值构造详细会话配置
  esr::session::EsrSessionConfig detailed_config{};
  detailed_config.mission.work_mode = esr::config::EsrWorkMode::kEsm;
  detailed_config.mission.scan.scan_rate_hz = 2.0f;
  detailed_config.policy.detection.use_profile_defaults = false;
  detailed_config.policy.detection.min_detect_snr_db = 8.0f;
  detailed_config.policy.detection.pfa = 1.0e-6f;
  detailed_config.policy.detection.pulse_count = 16U;
  detailed_config.policy.detection.threshold_scale = 1.0f;
  detailed_config.policy.detection.enable_statistical_detection = true;
  detailed_config.environment.scenario_config.preset = esr::config::EsrEnvironmentPreset::kStandard;

  // 3. Session construction
  esr::session::EsrSession session(config);

  // 4. CycleInput with a valid emitter
  esr::session::EsrCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  input.platform_altitude_m = 5000.0f;

  esr::session::EsrSceneEmitter emitter;
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
  input.scene.push_back(emitter);

  // 5. Input validation
  const esr::session::ValidationIssueList issues = esr::session::ValidateEsrCycleInput(input);
  if (esr::session::HasValidationError(issues)) {
    return 1;
  }

  // 6. StepWithResult
  const esr::session::EsrCycleResult result = session.StepWithResult(input);
  if (result.has_validation_error) {
    return 2;
  }

  // 7. Step (output-only)
  const esr::session::EsrOutputFrame step_frame = session.Step(input);

  // 8. Access three-channel output
  const std::size_t obs_count = result.output_frame.observation_output.observations.size();
  const std::size_t hyp_count = result.output_frame.emitter_output.hypotheses.size();
  const std::size_t assoc_count = result.output_frame.truth_evaluation_output.associations.size();
  const std::uint32_t cycle_index = result.output_frame.cycle_index;
  (void)obs_count;
  (void)hyp_count;
  (void)assoc_count;
  (void)cycle_index;

  // 9. RuntimeConfigBuilder: disable sensor
  const esr::session::EsrRuntimeConfigPatch disable_patch =
      esr::config::EsrRuntimeConfigBuilder().WithSensorEnabled(false).Build();
  session.ApplyRuntimeConfig(disable_patch);

  // 10. Step after sensor disabled — should return empty output
  esr::session::EsrCycleInput input_2 = input;
  input_2.cycle_index = 2U;
  const esr::session::EsrOutputFrame disabled_frame = session.Step(input_2);
  if (!disabled_frame.observation_output.observations.empty()) {
    return 3;
  }

  // 11. RuntimeConfigBuilder: re-enable sensor
  const esr::session::EsrRuntimeConfigPatch enable_patch =
      esr::config::EsrRuntimeConfigBuilder().WithSensorEnabled(true).Build();
  session.ApplyRuntimeConfig(enable_patch);

  // 12. RuntimeConfigBuilder: scan rate + work mode
  const esr::session::EsrRuntimeConfigPatch tune_patch =
      esr::config::EsrRuntimeConfigBuilder()
          .WithScanRateHz(2.0f)
          .WithWorkMode(esr::config::EsrWorkMode::kHgesm)
          .Build();
  session.ApplyRuntimeConfig(tune_patch);

  // 13. RuntimeConfigBuilder: explicit scan bounds
  const esr::session::EsrRuntimeConfigPatch window_patch =
      esr::config::EsrRuntimeConfigBuilder()
          .WithExplicitScanBoundsDeg(-45.0f, 45.0f, -15.0f, 15.0f)
          .Build();
  session.ApplyRuntimeConfig(window_patch);

  // 14. RuntimeConfigBuilder: reset to center-driven scan
  const esr::session::EsrRuntimeConfigPatch clear_window_patch =
      esr::config::EsrRuntimeConfigBuilder().SetUseExplicitScanBounds(false).Build();
  session.ApplyRuntimeConfig(clear_window_patch);

  // 15. RuntimeConfigBuilder: environment atmospheric config via environment runtime config
  esr::config::EsrAtmosphericPhysicsConfig atmospheric_physics;
  atmospheric_physics.enable_physical_model = true;
  atmospheric_physics.relative_humidity = 0.7f;
  const esr::session::EsrRuntimeConfigPatch env_patch =
      esr::config::EsrRuntimeConfigBuilder()
          .WithAtmosphericPhysicsConfig(atmospheric_physics)
          .Build();
  session.ApplyRuntimeConfig(env_patch);

  // 16. Final cycle
  esr::session::EsrCycleInput input_3 = input;
  input_3.cycle_index = 3U;
  const esr::session::EsrCycleResult result_3 = session.StepWithResult(input_3);
  if (result_3.has_validation_error) {
    return 4;
  }

  return 0;
}
