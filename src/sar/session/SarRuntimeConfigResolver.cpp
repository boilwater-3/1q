#include "sar/session/SarRuntimeConfigResolver.h"

#include "common/logging/ProjectLog.h"
#include "common/validation/ValidationUtils.h"

namespace sar {
namespace session {
namespace {

bool HasRequestedUpdate(const config::SarRuntimeConfigPatch& patch) {
  return patch.has_enable_raw_echo_generation ||
         patch.has_enable_l1_rda_imaging || patch.has_retain_raw_phase_history ||
         patch.has_retain_focused_image || patch.has_minimum_snr_db;
}

SarRuntimeConfigResolveResult RejectPatch(const config::SarSessionConfig& current_config,
                                          bool has_requested_update) {
  SarRuntimeConfigResolveResult rejected;
  rejected.next_config = current_config;
  rejected.has_requested_update = has_requested_update;
  rejected.is_valid = false;
  // 中译：运行期配置补丁因字段非法被拒绝，未应用任何变更。
  // 标识：补丁校验失败的整体拒绝出口——拒绝时不改动当前运行配置，
  //       排查具体非法字段请参考上方各分域校验逻辑。
  PROJECT_LOG_ERROR(
      "[SarSession] Runtime config patch rejected due to invalid fields; no changes applied.");
  return rejected;
}

}  // namespace

SarRuntimeConfigResolveResult ResolveSarRuntimeConfigPatch(
    const config::SarSessionConfig& current_config,
    const config::SarRuntimeConfigPatch& patch) {
  SarRuntimeConfigResolveResult resolved;
  resolved.next_config = current_config;
  const bool has_requested_update = HasRequestedUpdate(patch);

  if (patch.has_minimum_snr_db) {
    if (!oneq::common::validation::IsFinite(patch.minimum_snr_db)) {
      // 中译：最低信噪比补丁值非法（须有限），拒绝该补丁。
      // 标识：单字段校验失败——拒绝时本次补丁整体不生效。
      PROJECT_LOG_ERROR("[SarSession] Rejecting invalid minimum_snr_db; must be finite.");
      return RejectPatch(current_config, true);
    }
    resolved.next_config.policy.minimum_snr_db = patch.minimum_snr_db;
    resolved.policy_changed = true;
  }
  if (patch.has_enable_raw_echo_generation) {
    resolved.next_config.policy.enable_raw_echo_generation = patch.enable_raw_echo_generation;
    resolved.policy_changed = true;
  }
  if (patch.has_enable_l1_rda_imaging) {
    resolved.next_config.policy.enable_l1_rda_imaging = patch.enable_l1_rda_imaging;
    resolved.policy_changed = true;
  }
  if (patch.has_retain_raw_phase_history) {
    resolved.next_config.policy.retain_raw_phase_history = patch.retain_raw_phase_history;
    resolved.policy_changed = true;
  }
  if (patch.has_retain_focused_image) {
    resolved.next_config.policy.retain_focused_image = patch.retain_focused_image;
    resolved.policy_changed = true;
  }

  // L1 RDA 成像依赖 raw echo generation。基于 resolved.next_config 判定，
  // 使同一补丁内同时打开依赖项也能正确放行。
  if (resolved.next_config.policy.enable_l1_rda_imaging &&
      !resolved.next_config.policy.enable_raw_echo_generation) {
    // 中译：拒绝补丁：启用 L1 RDA 成像必须同时启用原始回波生成。
    // 标识：依赖校验——成像阶段依赖前置处理，未满足时补丁整体拒绝。
    PROJECT_LOG_ERROR(
        "[SarSession] Rejecting patch: enable_l1_rda_imaging requires raw echo generation.");
    return RejectPatch(current_config, true);
  }
  if (resolved.next_config.policy.retain_raw_phase_history &&
      !resolved.next_config.policy.enable_raw_echo_generation) {
    // 中译：拒绝补丁：保留原始相位历史必须同时启用原始回波生成。
    // 标识：依赖校验——保留相位依赖回波生成，未满足时补丁整体拒绝。
    PROJECT_LOG_ERROR(
        "[SarSession] Rejecting patch: retain_raw_phase_history requires "
        "enable_raw_echo_generation.");
    return RejectPatch(current_config, true);
  }

  resolved.has_requested_update = has_requested_update;
  if (has_requested_update) {
    // 中译：运行期配置补丁已应用（随后为各字段是否被本次补丁携带）。
    // 标识：补丁应用成功摘要——列出本次补丁实际修改了哪些处理开关，
    //       用于确认运行期变更已生效；无补丁时不输出。
    PROJECT_LOG_INFO(
        "[SarSession] runtime config patch applied: raw_echo={} "
        "l1_rda={} retain_raw={} retain_image={} min_snr_db={}",
        patch.has_enable_raw_echo_generation,
        patch.has_enable_l1_rda_imaging, patch.has_retain_raw_phase_history,
        patch.has_retain_focused_image, patch.has_minimum_snr_db);
  }
  return resolved;
}

}  // namespace session
}  // namespace sar
