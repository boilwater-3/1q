/**
 * @file eos_session_consumer.cpp
 * @brief 验证安装后 EOS 公共 API 路径可被外部工程编译链接。
 *
 * 覆盖要点：
 *   - EosSessionConfigBuilder 构造会话配置
 *   - EosCycleInput + EosTargetState 构造场景输入
 *   - EosInputValidation 输入校验
 *   - EosSession 构造、Step、StepWithResult 调用
 *   - EosOutputFrame 探测输出字段可访问
 *   - EosRuntimeConfigBuilder 热切换（工作模式、扫描率、SNR 门限、杂散光滤波、环境模型）
 */

#include <cstddef>
#include <cstdint>

#include "1q/electro_optical_sensor/common/EosOutputFrame.h"
#include "1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h"
#include "1q/electro_optical_sensor/config/EosSessionConfigBuilder.h"
#include "1q/electro_optical_sensor/core/context/EosCycleInput.h"
#include "1q/electro_optical_sensor/core/context/EosInputValidation.h"
#include "1q/electro_optical_sensor/core/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/core/session/EosSession.h"
#include "1q/electro_optical_sensor/environment/EosEnvironmentTypes.h"
#include "1q/electro_optical_sensor/foundation/EosRadiativeTransfer.h"

namespace eos = electro_optical_sensor;

int main() {
  // 1. SessionConfigBuilder
  eos::core::session::EosSessionConfig config =
      eos::config::EosSessionConfigBuilder()
          .WithWorkMode(eos::core::session::EosWorkMode::kFused)
          .WithScanRateDegPerSec(5.0f)
          .WithMinimumSnrDb(0.0f)
          .WithEnvironmentModelType(eos::environment::EosEnvironmentModelType::kSimplified)
          .Build();

  // 2. Session construction
  eos::core::session::EosSession session(config);

  // 3. CycleInput with a target
  eos::core::context::EosCycleInput input;
  input.cycle_index = 1U;
  input.dt_sec = 1.0f;
  input.solar_irradiance_w_m2 = 850.0f;
  input.solar_altitude_deg = 45.0f;
  input.atmospheric_transmittance = 0.8f;
  input.cloud_coverage_ratio = 0.2f;
  input.background_temperature_k = 289.0f;
  input.day_night_type = eos::core::context::DayNightType::kDay;
  input.platform_pose.position_m.z = 1200.0f;

  eos::core::context::EosTargetState target;
  target.target_id = 1U;
  target.range_m = 1500.0f;
  target.azimuth_deg = 0.0f;
  target.elevation_deg = 0.0f;
  target.apparent_temperature_k = 330.0f;
  target.emissivity = 0.92f;
  target.reflectance = 0.38f;
  target.projected_area_m2 = 4.0f;
  input.scene_targets.push_back(target);

  // 4. Input validation
  const eos::core::context::EosValidationIssueList issues =
      eos::core::context::ValidateEosCycleInput(input);
  if (eos::core::context::HasEosValidationError(issues)) {
    return 1;
  }

  // 5. StepWithResult
  const eos::core::session::EosCycleResult result = session.StepWithResult(input);
  if (result.has_validation_error) {
    return 2;
  }

  // 6. Step (output-only)
  const eos::common::EosOutputFrame step_frame = session.Step(input);

  // 7. Access detection output
  const std::size_t det_count = result.output_frame.detections.size();
  const std::uint32_t cycle_index = result.output_frame.cycle_index;
  const float scan_az = result.output_frame.scan_azimuth_deg;
  (void)det_count;
  (void)cycle_index;
  (void)scan_az;

  if (det_count > 0) {
    const eos::common::EosDetectionRecord& rec = result.output_frame.detections.front();
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

  // 8. RuntimeConfigBuilder: switch to infrared-only mode
  const eos::core::session::EosRuntimeConfigPatch ir_patch =
      eos::config::EosRuntimeConfigBuilder()
          .WithWorkMode(eos::core::session::EosWorkMode::kInfraredOnly)
          .Build();
  session.ApplyRuntimeConfig(ir_patch);

  // 9. Step after mode switch
  eos::core::context::EosCycleInput input_2 = input;
  input_2.cycle_index = 2U;
  const eos::common::EosOutputFrame ir_frame = session.Step(input_2);

  // 10. RuntimeConfigBuilder: change scan rate + SNR threshold
  const eos::core::session::EosRuntimeConfigPatch tune_patch =
      eos::config::EosRuntimeConfigBuilder()
          .WithScanRateDegPerSec(100.0f)
          .WithMinimumSnrDb(120.0f)
          .Build();
  session.ApplyRuntimeConfig(tune_patch);

  // 11. RuntimeConfigBuilder: enable straylight filter
  const eos::core::session::EosRuntimeConfigPatch straylight_patch =
      eos::config::EosRuntimeConfigBuilder().EnableStraylightFilter(true).Build();
  session.ApplyRuntimeConfig(straylight_patch);

  // 12. RuntimeConfigBuilder: switch environment model to advanced
  const eos::core::session::EosRuntimeConfigPatch env_patch =
      eos::config::EosRuntimeConfigBuilder()
          .WithEnvironmentModelType(eos::environment::EosEnvironmentModelType::kAdvanced)
          .WithAerosolDensityFactor(1.5f)
          .WithTurbulenceFactor(1.4f)
          .Build();
  session.ApplyRuntimeConfig(env_patch);

  // 13. RuntimeConfigBuilder: switch radiative transfer model
  const eos::core::session::EosRuntimeConfigPatch rt_patch =
      eos::config::EosRuntimeConfigBuilder()
          .WithRadiativeTransferModel(
              eos::foundation::radiative_transfer::RadiativeTransferModel::kAdaptivePathRadiance)
          .Build();
  session.ApplyRuntimeConfig(rt_patch);

  // 14. RuntimeConfigBuilder: change visible reference irradiance
  const eos::core::session::EosRuntimeConfigPatch vis_ref_patch =
      eos::config::EosRuntimeConfigBuilder().WithVisibleReferenceIrradianceWm2(1200.0f).Build();
  session.ApplyRuntimeConfig(vis_ref_patch);

  // 15. Final cycle
  eos::core::context::EosCycleInput input_3 = input;
  input_3.cycle_index = 3U;
  const eos::core::session::EosCycleResult result_3 = session.StepWithResult(input_3);
  if (result_3.has_validation_error) {
    return 3;
  }

  return 0;
}
