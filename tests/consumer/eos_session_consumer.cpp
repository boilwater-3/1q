/**
 * @file eos_session_consumer.cpp
 * @brief 验证安装后 EOS 公共 API 路径可被外部工程编译链接。
 *
 * 覆盖要点：
 *   - EosSessionConfigBuilder 语义档位常量配置构造
 *   - 直接字段赋值覆盖四域详细参数（hardware/mission/policy/environment）
 *   - EosCycleInput + EosSceneTarget 构造场景输入
 *   - EosInputValidation 输入校验
 *   - EosSession::Create 创建会话，Step、StepWithResult 调用
 *   - EosOutputFrame 探测输出字段可访问
 *   - EosRuntimeConfigBuilder 热切换（工作模式、扫描率、探测/杂散光/环境策略）
 */

#include <cstddef>
#include <cstdint>

#include "1q/electro_optical_sensor/config/EosProfileConstants.h"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "1q/electro_optical_sensor/session/EosSession.h"
#include "support/eos_enu_scene_helpers.h"

namespace eos = electro_optical_sensor;

int main() {
  // 1. 语义档位常量：mission / detection / environment
  const eos::config::EosSessionConfig semantic_config =
      eos::config::EosSessionConfigBuilder()
          .WithMission(eos::config::profiles::kWideAreaSearchMission)
          .WithEnvironmentPreset(eos::config::EosEnvironmentPreset::kStandard)
          .Build();
  auto config = semantic_config;
  config.mission.scan_rate_deg_per_sec = 5.0f;
  config.policy.detection.minimum_snr_db = 4.5f;  // 档位在前、微调在后 → 微调胜出
  config.policy.detection.detection_sensitivity_w = 0.9e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 720.0f;

  // 3. Session construction
  eos::session::EosSession session = eos::session::EosSession::Create(config);

  // 4. CycleInput with a target
  eos::session::EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 0.1f;
  input.platform_altitude_m = 1200.0f;

  eos::session::EosSceneTarget target;
  target.target_id = 1U;
  oneq::test_support::SetEosSphericalLook(&target, 1500.0f, 0.0f, 0.0f);
  target.appearance.apparent_temperature_k = 330.0f;
  target.appearance.emissivity = 0.92f;
  target.appearance.reflectance = 0.38f;
  target.appearance.projected_area_m2 = 4.0f;
  input.scene.push_back(target);

  // 5. Input validation
  const eos::session::EosIssueList issues = eos::session::ValidateEosCycleInput(input, 30.0f);
  if (eos::session::HasValidationError(issues)) {
    return 1;
  }

  // 6. StepWithResult
  const eos::session::EosCycleResult result = session.StepWithResult(input);
  if (eos::session::HasValidationError(result.issues)) {
    return 2;
  }

  // 7. Step (output-only)
  const eos::session::EosOutputFrame step_frame = session.Step(input);

  // 8. Access detection output
  const std::size_t det_count = result.output_frame.detections.size();
  const std::uint32_t cycle_index = result.output_frame.cycle_index;
  const float scan_az = result.output_frame.scan_azimuth_deg;
  (void)det_count;
  (void)cycle_index;
  (void)scan_az;

  if (det_count > 0) {
    const eos::output::EosDetectionRecord& rec = result.output_frame.detections.front();
    const float ir_snr = rec.infrared_snr_linear;
    const float vis_snr = rec.visible_snr_linear;
    const float fused_snr = rec.fused_snr_linear;
    const float fused_snr_db = rec.fused_snr_db;
    const bool detected = rec.detected;
    (void)ir_snr;
    (void)vis_snr;
    (void)fused_snr;
    (void)fused_snr_db;
    (void)detected;
  }

  // 9. RuntimeConfigBuilder: switch to infrared-only mode
  const eos::config::EosRuntimeConfigPatch ir_patch =
      eos::config::EosRuntimeConfigBuilder()
          .WithWorkMode(eos::config::EosWorkMode::kInfraredOnly)
          .Build();
  (void)session.TryApplyRuntimeConfig(ir_patch);

  // 10. Step after mode switch
  eos::session::EosCycleInput input_2 = input;
  input_2.cycle_index = 2U;
  const eos::session::EosOutputFrame ir_frame = session.Step(input_2);

  // 11. RuntimeConfigBuilder: change scan rate + SNR threshold
  const eos::config::EosRuntimeConfigPatch tune_patch =
      eos::config::EosRuntimeConfigBuilder()
          .WithScanRateDegPerSec(100.0f)
          .WithMinimumSnrDb(4.5f)
          .WithDetectionSensitivityW(0.8e-12f)
          .WithVisibleReferenceIrradianceWM2(700.0f)
          .Build();
  (void)session.TryApplyRuntimeConfig(tune_patch);

  // 12. RuntimeConfigBuilder: enable straylight filter
  const eos::config::EosRuntimeConfigPatch straylight_patch =
      eos::config::EosRuntimeConfigBuilder()
          .WithEnableStraylightFilter(true)
          .WithHoodInnerHalfAngleDeg(8.0f)
          .WithHoodOuterHalfAngleDeg(55.0f)
          .WithHoodMinSuppressionRatio(0.35f)
          .WithHoodMaxSuppressionRatio(0.95f)
          .Build();
  (void)session.TryApplyRuntimeConfig(straylight_patch);

  // 13. RuntimeConfigBuilder: switch the public environment preset.
  eos::config::EosEnvironmentScenarioConfig scenario;
  scenario.preset = eos::config::EosEnvironmentPreset::kTurbulent;
  const eos::config::EosRuntimeConfigPatch env_patch =
      eos::config::EosRuntimeConfigBuilder()
          .WithEnvironmentScenarioConfig(scenario)
          .Build();
  (void)session.TryApplyRuntimeConfig(env_patch);

  // 14. RuntimeConfigBuilder: select another public preset.
  scenario.preset = eos::config::EosEnvironmentPreset::kDusty;
  const eos::config::EosRuntimeConfigPatch rt_patch =
      eos::config::EosRuntimeConfigBuilder()
          .WithEnvironmentScenarioConfig(scenario)
          .Build();
  (void)session.TryApplyRuntimeConfig(rt_patch);

  // 15. RuntimeConfigBuilder: enable standard atmospheric physics.
  scenario.atmospheric_physics.enable_physical_model = true;
  scenario.atmospheric_physics.relative_humidity = 0.8f;
  const eos::config::EosRuntimeConfigPatch vis_ref_patch =
      eos::config::EosRuntimeConfigBuilder()
          .WithEnvironmentScenarioConfig(scenario)
          .Build();
  (void)session.TryApplyRuntimeConfig(vis_ref_patch);

  // 16. Final cycle
  eos::session::EosCycleInput input_3 = input;
  input_3.cycle_index = 3U;
  const eos::session::EosCycleResult result_3 = session.StepWithResult(input_3);
  if (eos::session::HasValidationError(result_3.issues)) {
    return 3;
  }

  return 0;
}
