/**
 * @file EosRuntimeConfigResolver.cpp
 * @brief 实现 EOS 运行期补丁解析，直接操作内部执行配置。
 */

#include "electro_optical_sensor/runtime/EosRuntimeConfigResolver.h"

#include <cmath>

#include "common/logging/ProjectLog.h"
#include "common/validation/ValidationUtils.h"
#include "electro_optical_sensor/runtime/EosPipelineConfigMapper.h"

namespace electro_optical_sensor {
namespace runtime {
namespace session {
namespace {

bool IsValidEnvironmentPreset(config::EosEnvironmentPreset preset) {
  switch (preset) {
    case config::EosEnvironmentPreset::kStandard:
    case config::EosEnvironmentPreset::kHumid:
    case config::EosEnvironmentPreset::kDusty:
    case config::EosEnvironmentPreset::kTurbulent:
    case config::EosEnvironmentPreset::kMaritime:
      return true;
  }
  return false;
}

bool IsValidMission(const config::EosMissionConfig& mission) {
  return oneq::common::validation::IsFinite(mission.scan_rate_deg_per_sec) &&
         mission.scan_rate_deg_per_sec > 0.0f &&
         oneq::common::validation::IsFinite(mission.frame_rate_hz) &&
         mission.frame_rate_hz > 0.0f &&
         oneq::common::validation::IsFinite(mission.horizontal_fov_deg) &&
         mission.horizontal_fov_deg > 0.0f &&
         oneq::common::validation::IsFinite(mission.vertical_fov_deg) &&
         mission.vertical_fov_deg > 0.0f &&
         oneq::common::validation::IsFinite(mission.scan_start_az_deg) &&
         oneq::common::validation::IsFinite(mission.scan_end_az_deg) &&
         oneq::common::validation::IsFinite(mission.scan_center_el_deg) &&
         oneq::common::validation::IsFinite(mission.boresight_depression_deg);
}

bool IsValidDetectionPolicy(const config::EosDetectionPolicyConfig& detection) {
  return oneq::common::validation::IsFinite(detection.minimum_snr_db) &&
         oneq::common::validation::IsFinite(detection.detection_sensitivity_w) &&
         detection.detection_sensitivity_w > 0.0f &&
         oneq::common::validation::IsFinite(detection.visible_reference_irradiance_w_m2) &&
         detection.visible_reference_irradiance_w_m2 > 0.0f;
}

bool IsValidStrayLightPolicy(const config::EosStrayLightPolicyConfig& stray_light) {
  if (!oneq::common::validation::IsFinite(stray_light.hood_inner_half_angle_deg) ||
      !oneq::common::validation::IsFinite(stray_light.hood_outer_half_angle_deg) ||
      !oneq::common::validation::IsFinite(stray_light.hood_min_suppression_ratio) ||
      !oneq::common::validation::IsFinite(stray_light.hood_max_suppression_ratio)) {
    return false;
  }
  if (stray_light.hood_inner_half_angle_deg <= 0.0f ||
      stray_light.hood_outer_half_angle_deg <= stray_light.hood_inner_half_angle_deg) {
    return false;
  }
  if (stray_light.hood_min_suppression_ratio < 0.0f ||
      stray_light.hood_min_suppression_ratio > 1.0f ||
      stray_light.hood_max_suppression_ratio < 0.0f ||
      stray_light.hood_max_suppression_ratio > 1.0f) {
    return false;
  }
  return stray_light.hood_max_suppression_ratio >= stray_light.hood_min_suppression_ratio;
}

bool IsValidPolicy(const config::EosPolicyConfig& policy) {
  return IsValidDetectionPolicy(policy.detection) &&
         IsValidStrayLightPolicy(policy.stray_light);
}

bool IsValidEnvironmentPatch(
    const config::EosEnvironmentRuntimeConfigPatch& environment_patch) {
  if (environment_patch.has_scenario_config) {
    const config::EosEnvironmentScenarioConfig& scenario =
        environment_patch.scenario_config;
    if (!IsValidEnvironmentPreset(scenario.preset)) {
      return false;
    }
    const config::EosAtmosphericPhysicsConfig& atmosphere =
        scenario.atmospheric_physics;
    if (atmosphere.enable_physical_model &&
        (!oneq::common::validation::IsFinite(atmosphere.pressure_hpa) ||
         atmosphere.pressure_hpa <= 0.0f ||
         !oneq::common::validation::IsFinite(atmosphere.temperature_k) ||
         atmosphere.temperature_k <= 0.0f ||
         !oneq::common::validation::IsFinite(atmosphere.relative_humidity) ||
         atmosphere.relative_humidity < 0.0f ||
         atmosphere.relative_humidity > 1.0f)) {
      return false;
    }
  }
  return true;
}

EosRuntimeConfigResolveResult RejectPatch(
    const config::execution::EosInternalExecutionConfig& current_config,
    bool has_requested_update) {
  EosRuntimeConfigResolveResult rejected;
  rejected.next_config = current_config;
  rejected.has_requested_update = has_requested_update;
  rejected.is_valid = false;
  // 中译：运行期配置补丁因字段非法被拒绝，未应用任何变更。
  // 标识：补丁校验失败的整体拒绝出口——拒绝时不改动当前运行配置，
  //       排查具体非法字段请参考上方各分域校验函数。
  PROJECT_LOG_ERROR(
      "[EosSession] Runtime config patch rejected due to invalid fields; no changes applied.");
  return rejected;
}

}  // namespace

EosRuntimeConfigResolveResult ResolveEosRuntimeConfigPatch(
    const config::execution::EosInternalExecutionConfig& current_config,
    const ::electro_optical_sensor::config::EosRuntimeConfigPatch& patch) {
  EosRuntimeConfigResolveResult resolved;
  resolved.next_config = current_config;
  const bool has_requested_update =
      patch.has_mission || patch.has_policy || patch.has_environment ||
      patch.has_work_mode || patch.has_scan_rate_deg_per_sec || patch.has_frame_rate_hz ||
      patch.has_sensor_enabled;

  if (patch.has_mission) {
    if (!IsValidMission(patch.mission)) {
      // 中译：任务（mission）补丁值非法，拒绝该补丁。
      // 标识：任务域校验失败——扫描速率/帧率/视场等字段必须有限且为正，
      //       拒绝时本次补丁整体不生效，当前配置保持不变。
      PROJECT_LOG_ERROR(
          "[EosSession] Rejecting mission patch because mission values are invalid.");
      return RejectPatch(current_config, true);
    }
    // 电源单源：mission 域无电源字段（COMMON-OQ-4，见 contract.md §电源状态单源契约）。
    resolved.next_config.scan = patch.mission;
    resolved.reset_scan_phase = true;
  }

  if (patch.has_policy) {
    if (!IsValidPolicy(patch.policy)) {
      // 中译：探测策略（policy）补丁值非法，拒绝该补丁。
      // 标识：策略域校验失败——检测门限/杂散光抑制参数非法，
      //       拒绝时本次补丁整体不生效，当前配置保持不变。
      PROJECT_LOG_ERROR(
          "[EosSession] Rejecting policy patch because policy values are invalid.");
      return RejectPatch(current_config, true);
    }
    resolved.next_config.detection = patch.policy.detection;
    resolved.next_config.stray_light = patch.policy.stray_light;
  }

  if (patch.has_environment) {
    if (!IsValidEnvironmentPatch(patch.environment)) {
      // 中译：环境（environment）补丁值非法，拒绝该补丁。
      // 标识：环境域校验失败——场景预设或大气物理参数非法，
      //       拒绝时本次补丁整体不生效，当前配置保持不变。
      PROJECT_LOG_ERROR(
          "[EosSession] Rejecting environment patch because environment values are invalid.");
      return RejectPatch(current_config, true);
    }

    if (patch.environment.has_scenario_config) {
      const config::execution::EnvironmentConfig model_config =
          config::BuildModelConfigFromScenario(patch.environment.scenario_config);
      ApplyEnvironmentModelToInternal(model_config, &resolved.next_config);
    }
  }

  if (patch.has_work_mode) {
    resolved.next_config.scan.work_mode = patch.work_mode;
  }

  if (patch.has_scan_rate_deg_per_sec) {
    if (!oneq::common::validation::IsFinite(patch.scan_rate_deg_per_sec) ||
        patch.scan_rate_deg_per_sec <= 0.0f) {
      // 中译：扫描速率补丁值非法（须为有限正数），拒绝该补丁。
      // 标识：单字段校验失败——扫描速率必须有限且大于 0，
      //       拒绝时本次补丁整体不生效，当前配置保持不变。
      PROJECT_LOG_ERROR(
          "[EosSession] Rejecting invalid scan_rate_deg_per_sec={}; "
          "must be finite and positive.",
          patch.scan_rate_deg_per_sec);
      return RejectPatch(current_config, true);
    }
    resolved.next_config.scan.scan_rate_deg_per_sec = patch.scan_rate_deg_per_sec;
    resolved.reset_scan_phase = true;
  }

  if (patch.has_frame_rate_hz) {
    if (!oneq::common::validation::IsFinite(patch.frame_rate_hz) ||
        patch.frame_rate_hz <= 0.0f) {
      // 中译：帧率补丁值非法（须为有限正数），拒绝该补丁。
      // 标识：单字段校验失败——帧率必须有限且大于 0，
      //       拒绝时本次补丁整体不生效，当前配置保持不变。
      PROJECT_LOG_ERROR(
          "[EosSession] Rejecting invalid frame_rate_hz={}; "
          "must be finite and positive.",
          patch.frame_rate_hz);
      return RejectPatch(current_config, true);
    }
    resolved.next_config.scan.frame_rate_hz = patch.frame_rate_hz;
  }

  if (patch.has_sensor_enabled) {
    resolved.next_config.sensor_enabled = patch.sensor_enabled;
  }

  resolved.has_requested_update = has_requested_update;
  if (has_requested_update) {
    // 中译：运行期配置补丁已应用（随后为各字段是否被本次补丁携带）。
    // 标识：补丁应用成功摘要——列出本次补丁实际修改了哪些域，
    //       用于确认运行期变更已生效；无补丁时不输出。
    PROJECT_LOG_INFO(
        "[EosSession] runtime config patch applied: mission={} policy={} environment={} "
        "work_mode={} scan_rate={} frame_rate={} sensor_enabled={}",
        patch.has_mission, patch.has_policy, patch.has_environment,
        patch.has_work_mode, patch.has_scan_rate_deg_per_sec, patch.has_frame_rate_hz,
        patch.has_sensor_enabled);
  }
  return resolved;
}

}  // namespace session
}  // namespace runtime
}  // namespace electro_optical_sensor
