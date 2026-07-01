#include "sar/session/SarRuntimeConfigResolver.h"

#include "common/logging/ProjectLog.h"
#include "common/validation/ValidationUtils.h"

namespace sar {
namespace session {
namespace {

bool HasRequestedUpdate(const config::SarRuntimeConfigPatch& patch) {
  return patch.has_enable_raw_echo_generation || patch.has_enable_range_compression ||
         patch.has_enable_l1_rda_imaging || patch.has_retain_raw_phase_history ||
         patch.has_retain_focused_image || patch.has_minimum_snr_db;
}

SarRuntimeConfigResolveResult RejectPatch(const config::SarSessionConfig& current_config,
                                          bool has_requested_update) {
  SarRuntimeConfigResolveResult rejected;
  rejected.next_config = current_config;
  rejected.has_requested_update = has_requested_update;
  rejected.is_valid = false;
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
    if (!oneq::internal::validation::IsFinite(patch.minimum_snr_db)) {
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
  if (patch.has_enable_range_compression) {
    resolved.next_config.policy.enable_range_compression = patch.enable_range_compression;
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

  // L1 RDA 成像依赖 raw echo generation：原为 step-time gate
  // （SarRuntimeConfigValidation.cpp:54-58），此处前置到 apply 时刻。
  // 基于 resolved.next_config 判定，使同一补丁内同时打开两者也能正确放行。
  if (resolved.next_config.policy.enable_l1_rda_imaging &&
      !resolved.next_config.policy.enable_raw_echo_generation) {
    PROJECT_LOG_ERROR(
        "[SarSession] Rejecting patch: enable_l1_rda_imaging requires enable_raw_echo_generation.");
    return RejectPatch(current_config, true);
  }

  resolved.has_requested_update = has_requested_update;
  if (has_requested_update) {
    PROJECT_LOG_INFO(
        "[SarSession] runtime config patch applied: raw_echo={} range_compression={} "
        "l1_rda={} retain_raw={} retain_image={} min_snr_db={}",
        patch.has_enable_raw_echo_generation, patch.has_enable_range_compression,
        patch.has_enable_l1_rda_imaging, patch.has_retain_raw_phase_history,
        patch.has_retain_focused_image, patch.has_minimum_snr_db);
  }
  return resolved;
}

}  // namespace session
}  // namespace sar
