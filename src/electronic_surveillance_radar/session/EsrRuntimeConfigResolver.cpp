#include "electronic_surveillance_radar/session/EsrRuntimeConfigResolver.h"

#include <algorithm>
#include "electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
#include "electronic_surveillance_radar/session/EsrConfigDomainValidation.h"
#include "electronic_surveillance_radar/session/EsrResolutionRules.h"
#include "common/logging/ProjectLog.h"
#include "common/validation/ValidationUtils.h"

namespace electronic_surveillance_radar {
namespace session {

using resolution_rules::ApplyScanPolicy;
using resolution_rules::ApplyWorkModeAdjustment;

namespace {

EsrRuntimeConfigResolveResult RejectPatch(const EsrInternalExecutionConfig& current_config,
                                          bool has_requested_update,
                                          EsrRuntimeConfigApplyStatus status) {
  EsrRuntimeConfigResolveResult rejected;
  rejected.next_config = current_config;
  rejected.status = status;
  rejected.has_requested_update = has_requested_update;
  rejected.is_valid = false;
  return rejected;
}

void ApplyEnvironmentRuntimePatch(const config::EsrEnvironmentRuntimeConfigPatch& env_patch,
                                  EsrInternalExecutionConfig* resolved, bool* env_changed) {
  if (env_patch.has_atmospheric_physics) {
    resolved->environment.atmospheric_physics = env_patch.atmospheric_physics;
    *env_changed = true;
  }
  if (env_patch.has_propagation_profile) {
    resolved->environment.propagation_profile = env_patch.propagation_profile;
    *env_changed = true;
  }
  if (env_patch.has_clutter_density) {
    resolved->environment.clutter_density = env_patch.clutter_density;
    *env_changed = true;
  }
  if (env_patch.has_spectrum_occupancy_ratio) {
    resolved->environment.spectrum_occupancy_ratio = env_patch.spectrum_occupancy_ratio;
    *env_changed = true;
  }
  if (env_patch.has_atmospheric_observation) {
    resolved->environment.atmospheric_observation = env_patch.atmospheric_observation;
    *env_changed = true;
  }
}

}  // namespace

EsrRuntimeConfigResolveResult ResolveEsrRuntimeConfigPatch(
    const EsrInternalExecutionConfig& current_config, const config::EsrRuntimeConfigPatch& patch) {
  EsrRuntimeConfigResolveResult resolved;
  resolved.next_config = current_config;
  resolved.status = EsrRuntimeConfigApplyStatus::kNoRequestedUpdate;
  bool has_requested_update = false;
  bool scan_policy_changed = false;
  bool work_mode_or_policy_changed = false;

  // ---- Phase 1: 整块域覆盖（先于叶子） ----

  if (patch.has_mission) {
    has_requested_update = true;
    // 电源单源：mission 域无电源字段（COMMON-OQ-4，见 contract.md §电源状态单源契约）。
    resolved.next_config.mission = patch.mission;
    scan_policy_changed = true;
    work_mode_or_policy_changed = true;
    resolved.runtime_config_changed = true;
    resolved.pipeline_config_changed = true;
  }

  if (patch.has_policy) {
    has_requested_update = true;
    resolved.next_config.base_detection = patch.policy.detection;
    work_mode_or_policy_changed = true;
    resolved.pipeline_config_changed = true;
  }

  if (patch.has_environment) {
    has_requested_update = true;
    ApplyEnvironmentRuntimePatch(patch.environment, &resolved.next_config,
                                 &resolved.environment_model_config_changed);
  }

  // ---- Phase 2: 叶子快捷覆盖（覆盖整块域的结果） ----

  if (patch.has_sensor_enabled) {
    resolved.next_config.sensor_enabled = patch.sensor_enabled;
    resolved.runtime_config_changed = true;
    has_requested_update = true;
  }

  if (patch.has_work_mode) {
    resolved.next_config.mission.work_mode = patch.work_mode;
    work_mode_or_policy_changed = true;
    resolved.pipeline_config_changed = true;
    has_requested_update = true;
  }

  if (patch.has_scan_rate_hz) {
    has_requested_update = true;
    resolved.next_config.mission.scan.scan_rate_hz = patch.scan_rate_hz;
    scan_policy_changed = true;
    resolved.runtime_config_changed = true;
  }

  if (patch.has_scan_start_position) {
    resolved.next_config.mission.scan.scan_start_position = patch.scan_start_position;
    scan_policy_changed = true;
    resolved.pipeline_config_changed = true;
    has_requested_update = true;
  }
  if (patch.has_scan_sequence) {
    resolved.next_config.mission.scan.scan_sequence = patch.scan_sequence;
    scan_policy_changed = true;
    resolved.pipeline_config_changed = true;
    has_requested_update = true;
  }
  if (patch.has_scan_center_az_deg) {
    has_requested_update = true;
    resolved.next_config.mission.scan.scan_center_az_deg = patch.scan_center_az_deg;
    resolved.next_config.mission.scan.use_explicit_scan_bounds = false;
    scan_policy_changed = true;
    resolved.pipeline_config_changed = true;
  }
  if (patch.has_scan_center_el_deg) {
    has_requested_update = true;
    resolved.next_config.mission.scan.scan_center_el_deg = patch.scan_center_el_deg;
    resolved.next_config.mission.scan.use_explicit_scan_bounds = false;
    scan_policy_changed = true;
    resolved.pipeline_config_changed = true;
  }
  if (patch.has_explicit_scan_bounds) {
    has_requested_update = true;
    scan_policy_changed = true;
    resolved.pipeline_config_changed = true;
    resolved.next_config.mission.scan.use_explicit_scan_bounds = patch.explicit_scan_bounds.enabled;
    if (patch.explicit_scan_bounds.enabled) {
      const auto& sb = patch.explicit_scan_bounds;
      resolved.next_config.mission.scan.scan_start_az_deg = sb.scan_start_az_deg;
      resolved.next_config.mission.scan.scan_end_az_deg = sb.scan_end_az_deg;
      resolved.next_config.mission.scan.scan_start_el_deg = sb.scan_start_el_deg;
      resolved.next_config.mission.scan.scan_end_el_deg = sb.scan_end_el_deg;
    }
  }

  // ---- Phase 3: 对整域与叶子合并后的最终领域状态统一校验并解析 ----

  if (!config_validation::IsValidMissionEnums(resolved.next_config.mission)) {
    return RejectPatch(current_config, has_requested_update,
                       EsrRuntimeConfigApplyStatus::kRejectedInvalidMission);
  }
  if (!config_validation::IsValidDetectionPolicy(
          resolved.next_config.base_detection)) {
    return RejectPatch(current_config, has_requested_update,
                       EsrRuntimeConfigApplyStatus::kRejectedInvalidPolicy);
  }
  if (!config_validation::IsValidEnvironment(
          resolved.next_config.environment)) {
    return RejectPatch(current_config, has_requested_update,
                       EsrRuntimeConfigApplyStatus::kRejectedInvalidEnvironment);
  }

  if (scan_policy_changed) {
    const config::EsrScanPolicyConfig& scan = resolved.next_config.mission.scan;
    if (!oneq::common::validation::IsFinite(scan.scan_rate_hz) || scan.scan_rate_hz <= 0.0f) {
      // 中译：扫描速率最终值非法（须为有限正数），拒绝运行期补丁。
      // 标识：整域与叶子合并后的最终校验失败——拒绝时补丁不生效，
      //       当前配置保持不变。
      PROJECT_LOG_ERROR(
          "[EsrSession] Rejecting runtime config patch due to invalid final scan_rate_hz={}; "
          "must be finite and positive.",
          scan.scan_rate_hz);
      return RejectPatch(current_config, true,
                         EsrRuntimeConfigApplyStatus::kRejectedInvalidScanRate);
    }
    if (scan.use_explicit_scan_bounds) {
      if (!oneq::common::validation::IsFinite(scan.scan_start_az_deg) ||
          !oneq::common::validation::IsFinite(scan.scan_end_az_deg) ||
          !oneq::common::validation::IsFinite(scan.scan_start_el_deg) ||
          !oneq::common::validation::IsFinite(scan.scan_end_el_deg) ||
          scan.scan_start_az_deg >= scan.scan_end_az_deg ||
          scan.scan_start_el_deg >= scan.scan_end_el_deg) {
        // 中译：显式扫描边界最终值非法（须有限且起点小于终点），拒绝运行期补丁。
        // 标识：边界扫描模式下最终校验失败——拒绝时补丁不生效，
        //       当前配置保持不变。
        PROJECT_LOG_ERROR(
            "[EsrSession] Rejecting runtime config patch due to invalid final explicit scan "
            "bounds.");
        return RejectPatch(current_config, true,
                           EsrRuntimeConfigApplyStatus::kRejectedInvalidExplicitScanBounds);
      }
    } else {
      if (!oneq::common::validation::IsFinite(scan.scan_center_az_deg)) {
        return RejectPatch(current_config, true,
                           EsrRuntimeConfigApplyStatus::kRejectedInvalidScanCenterAz);
      }
      if (!oneq::common::validation::IsFinite(scan.scan_center_el_deg)) {
        return RejectPatch(current_config, true,
                           EsrRuntimeConfigApplyStatus::kRejectedInvalidScanCenterEl);
      }
    }
    ApplyScanPolicy(resolved.next_config.hardware, scan, &resolved.next_config.resolved_scan);
  }

  if (work_mode_or_policy_changed) {
    resolved.next_config.detection = resolved.next_config.base_detection;
    ApplyWorkModeAdjustment(resolved.next_config.mission.work_mode,
                            &resolved.next_config.detection);
  }

  resolved.has_requested_update = has_requested_update;
  if (has_requested_update) {
    resolved.status = EsrRuntimeConfigApplyStatus::kApplied;
    // 中译：运行期配置补丁已应用（随后为各字段是否被本次补丁携带）。
    // 标识：补丁应用成功摘要——列出本次补丁实际修改了哪些域，
    //       用于确认运行期变更已生效；无补丁时不输出。
    PROJECT_LOG_INFO(
        "[EsrSession] runtime config patch applied: mission={} policy={} env={} sensor_enabled={} "
        "work_mode={} scan_rate={} scan_bounds={}",
        patch.has_mission, patch.has_policy, patch.has_environment, patch.has_sensor_enabled,
        patch.has_work_mode, patch.has_scan_rate_hz, patch.has_explicit_scan_bounds);
  }
  return resolved;
}

}  // namespace session
}  // namespace electronic_surveillance_radar
