/**
 * @file eos_session_consumer.cpp
 * @brief 验证安装后 EOS 公共 API 路径可被外部工程编译链接。
 *
 * 覆盖要点：
 *   - EosSessionConfigBuilder 语义化会话配置构造
 *   - 直接字段赋值覆盖四域详细参数（hardware/mission/policy/environment）
 *   - EosCycleInput + EosSceneTarget 构造场景输入
 *   - EosInputValidation 输入校验
 *   - EosSessionFactory 创建会话，Step、StepWithResult 调用
 *   - EosOutputFrame 探测输出字段可访问
 *   - EosRuntimeConfigBuilder 热切换（工作模式、扫描率、探测/杂散光/环境策略）
 */

#include <cstddef>
#include <cstdint>

#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/environment/EosEnvironmentTypes.h"
#include "1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h"
#include "1q/electro_optical_sensor/session/EosCycleInput.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosInputValidation.h"
#include "1q/electro_optical_sensor/session/EosSession.h"

namespace eos = electro_optical_sensor;

int main() {
  // 1. 语义 Builder：mission / detection / environment
  const eos::session::EosSessionConfig semantic_config =
      eos::config::EosSessionConfigBuilder()
          .Mission()
          .WithWorkMode(eos::config::EosWorkMode::kFused)
          .End()
          .Detection()
          .WithDetectionProfile(eos::config::EosDetectionProfile::kAggressive)
          .End()
          .Environment()
          .WithEnvironmentModelType(eos::environment::EosEnvironmentModelType::kSimplified)
          .End()
          .Build();
  // 2. 直接字段赋值覆盖四域详细参数
  auto config = semantic_config;
  config.mission.scan_rate_deg_per_sec = 5.0f;
  config.policy.detection.use_profile_defaults = false;
  config.policy.detection.minimum_snr_db = 4.5f;
  config.policy.detection.detection_sensitivity_w = 0.9e-12f;
  config.policy.detection.visible_reference_irradiance_w_m2 = 720.0f;

  // 3. Session construction
  eos::session::EosSession session = eos::session::EosSessionFactory::Create(config);

  // 4. CycleInput with a target
  eos::session::EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  input.environment.solar_irradiance_w_m2 = 850.0f;
  input.environment.solar_altitude_deg = 45.0f;
  input.environment.cloud_coverage_ratio = 0.2f;
  input.environment.background_temperature_k = 289.0f;
  input.environment.day_night_type = eos::session::DayNightType::kDay;
  input.platform_pose.position_m.z = 1200.0f;

  eos::session::EosSceneTarget target;
  target.target_id = 1U;
  target.range_m = 1500.0f;
  target.azimuth_deg = 0.0f;
  target.elevation_deg = 0.0f;
  target.appearance.apparent_temperature_k = 330.0f;
  target.appearance.emissivity = 0.92f;
  target.appearance.reflectance = 0.38f;
  target.appearance.projected_area_m2 = 4.0f;
  input.scene.push_back(target);

  // 5. Input validation
  const eos::session::ValidationIssueList issues = eos::session::ValidateEosCycleInput(input);
  if (eos::session::HasValidationError(issues)) {
    return 1;
  }

  // 6. StepWithResult
  const eos::session::EosCycleResult result = session.StepWithResult(input);
  if (result.has_validation_error) {
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
  const eos::session::EosRuntimeConfigPatch ir_patch =
      eos::config::EosRuntimeConfigBuilder()
          .WithWorkMode(eos::config::EosWorkMode::kInfraredOnly)
          .Build();
  session.ApplyRuntimeConfig(ir_patch);

  // 10. Step after mode switch
  eos::session::EosCycleInput input_2 = input;
  input_2.cycle_index = 2U;
  const eos::session::EosOutputFrame ir_frame = session.Step(input_2);

  // 11. RuntimeConfigBuilder: change scan rate + SNR threshold
  const eos::session::EosRuntimeConfigPatch tune_patch =
      eos::config::EosRuntimeConfigBuilder()
          .WithScanRateDegPerSec(100.0f)
          .WithDetectionProfile(eos::config::EosDetectionProfile::kAggressive)
          .Build();
  session.ApplyRuntimeConfig(tune_patch);

  // 12. RuntimeConfigBuilder: enable straylight filter
  const eos::session::EosRuntimeConfigPatch straylight_patch =
      eos::config::EosRuntimeConfigBuilder()
          .WithStrayLightProfile(eos::config::EosStrayLightProfile::kEnhancedHood)
          .Build();
  session.ApplyRuntimeConfig(straylight_patch);

  // 13. RuntimeConfigBuilder: switch environment model to advanced
  const eos::session::EosRuntimeConfigPatch env_patch =
      eos::config::EosRuntimeConfigBuilder()
          .WithEnvironmentModelType(eos::environment::EosEnvironmentModelType::kAdvanced)
          .WithEnvironmentDetails(
              eos::foundation::radiative_transfer::RadiativeTransferModel::kAdaptivePathRadiance,
              1.3f, 1.8f)
          .Build();
  session.ApplyRuntimeConfig(env_patch);

  // 14. RuntimeConfigBuilder: tune environment model details
  const eos::session::EosRuntimeConfigPatch rt_patch =
      eos::config::EosRuntimeConfigBuilder()
          .WithEnvironmentDetails(
              eos::foundation::radiative_transfer::RadiativeTransferModel::kAdaptivePathRadiance,
              2.0f, 1.2f)
          .Build();
  session.ApplyRuntimeConfig(rt_patch);

  // 15. RuntimeConfigBuilder: change environment model details again
  const eos::session::EosRuntimeConfigPatch vis_ref_patch =
      eos::config::EosRuntimeConfigBuilder()
          .WithEnvironmentDetails(
              eos::foundation::radiative_transfer::RadiativeTransferModel::kHumidityWeighted, 1.1f,
              1.1f)
          .Build();
  session.ApplyRuntimeConfig(vis_ref_patch);

  // 16. Final cycle
  eos::session::EosCycleInput input_3 = input;
  input_3.cycle_index = 3U;
  const eos::session::EosCycleResult result_3 = session.StepWithResult(input_3);
  if (result_3.has_validation_error) {
    return 3;
  }

  return 0;
}
