// @file SarInputValidation.cpp
// @brief Implementation of SAR cycle input validation.

#include "1q/sar/session/SarInputValidation.h"

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "1q/sar/session/SarIssueCodes.h"
#include "common/validation/ValidationUtils.h"

namespace sar {
namespace session {

namespace {

bool IsFinitePlatform(const SarPlatformState& platform) {
  return std::isfinite(platform.time_s) && std::isfinite(platform.latitude_deg) &&
         std::isfinite(platform.longitude_deg) && std::isfinite(platform.altitude_m) &&
         std::isfinite(platform.velocity_north_mps) &&
         std::isfinite(platform.velocity_east_mps) &&
         std::isfinite(platform.velocity_down_mps) && std::isfinite(platform.roll_deg) &&
         std::isfinite(platform.pitch_deg) && std::isfinite(platform.yaw_deg);
}

bool IsFiniteTarget(const SarPointTarget& target) {
  return std::isfinite(target.latitude_deg) && std::isfinite(target.longitude_deg) &&
         std::isfinite(target.altitude_m) && std::isfinite(target.radar_cross_section_dbsm);
}

bool IsFinitePulse(const SarRawIqFrame::PulseState& pulse) {
  return std::isfinite(pulse.time_s) && std::isfinite(pulse.position_x_m) &&
         std::isfinite(pulse.position_y_m) && std::isfinite(pulse.position_z_m) &&
         std::isfinite(pulse.velocity_x_mps) && std::isfinite(pulse.velocity_y_mps) &&
         std::isfinite(pulse.velocity_z_mps);
}

// 统一问题列表模型（规则 14）：校验问题 code 引用 SarIssueCodes.h 常量
// （"sar.validation.<snake_case>" 全集单一事实来源）；phase 固定为
// kInputValidation；location/field 为可选定位。
SarIssue MakeIssue(SarIssueSeverity severity, const char* code,
                   oneq::foundation::ValidationLocationKind location_kind,
                   std::size_t entity_index, const std::string& field,
                   const std::string& message) {
  SarIssue issue = oneq::common::validation::MakeLocatedIssue<
      SarIssue, oneq::foundation::ValidationLocation>(severity, code, location_kind,
                                                     entity_index, field, message);
  issue.phase = SarIssuePhase::kInputValidation;
  return issue;
}

}  // namespace

SarIssueList ValidateSarCycleInput(const SarCycleInput& input) {
  SarIssueList issues;
  const auto add = [&issues](SarIssueSeverity severity, const char* code,
                             oneq::foundation::ValidationLocationKind kind, std::size_t index,
                             const char* field, const char* message) {
    issues.push_back(
        MakeIssue(severity, code, kind, index, field, message));
  };

  // 周期步长
  if (!std::isfinite(input.dt_sec)) {
    add(SarIssueSeverity::kError, codes::kNonFiniteCycleDeltaTime,
        oneq::foundation::ValidationLocationKind::kGlobal, static_cast<std::size_t>(-1), "dt_sec",
        "Cycle delta time must be finite.");
  } else if (input.dt_sec <= 0.0f) {
    add(SarIssueSeverity::kError, codes::kInvalidCycleDeltaTime,
        oneq::foundation::ValidationLocationKind::kGlobal, static_cast<std::size_t>(-1), "dt_sec",
        "Cycle delta time must be positive.");
  }

  // 平台字段有限性
  if (!IsFinitePlatform(input.platform)) {
    add(SarIssueSeverity::kError, codes::kNonFinitePlatformField,
        oneq::foundation::ValidationLocationKind::kPlatform, static_cast<std::size_t>(-1),
        "platform", "Platform contains non-finite numeric field.");
  }

  // 点目标字段有限性
  for (std::size_t i = 0; i < input.point_targets.size(); ++i) {
    if (!IsFiniteTarget(input.point_targets[i])) {
      add(SarIssueSeverity::kError, codes::kNonFiniteTargetField,
          oneq::foundation::ValidationLocationKind::kSceneEntity, i, "point_targets",
          "Point target contains non-finite numeric field.");
    }
  }

  // 外部脉冲状态（仅在提供时校验）
  const auto& pulses = input.raw_iq.pulse_states;
  if (!pulses.empty()) {
    for (std::size_t i = 0; i < pulses.size(); ++i) {
      if (!IsFinitePulse(pulses[i])) {
        add(SarIssueSeverity::kError, codes::kNonFinitePulseField,
            oneq::foundation::ValidationLocationKind::kSceneEntity, i, "raw_iq.pulse_states",
            "Pulse state contains non-finite numeric field.");
        continue;  // 序列连续性检查跳过非有限脉冲
      }
      if (i > 0) {
        const auto& prev = pulses[i - 1];
        const auto& curr = pulses[i];
        if (curr.pulse_id != prev.pulse_id + 1U || curr.time_s <= prev.time_s) {
          add(SarIssueSeverity::kError, codes::kInvalidPulseSequence,
              oneq::foundation::ValidationLocationKind::kSceneEntity, i, "raw_iq.pulse_states",
              "Pulse id must be contiguous and time must be monotonically increasing.");
        }
      }
    }
  }

  return issues;
}

bool HasValidationError(const SarIssueList& issues) {
  // RED-1 收敛：统一判定逻辑在 common::validation::HasValidationPhaseError
  // （phase == kInputValidation && severity == kError）。
  return oneq::common::validation::HasValidationPhaseError(
      issues, &SarIssue::phase, &SarIssue::severity);
}

bool AreSarHardwareAndMissionFieldsValid(const config::SarSessionConfig& config) {
  if (config.hardware.bandwidth_hz <= 0.0 || config.hardware.sample_rate_hz <= 0.0 ||
      config.hardware.carrier_frequency_hz <= 0.0 ||
      config.hardware.pulse_repetition_frequency_hz <= 0.0 ||
      config.hardware.antenna_length_m <= 0.0 ||
      config.mission.platform_speed_mps <= 0.0 || config.mission.nominal_slant_range_m <= 0.0 ||
      config.mission.range_sample_count == 0U || config.mission.azimuth_pulse_count == 0U ||
      config.mission.desired_ground_range_resolution_m <= 0.0 ||
      config.mission.desired_azimuth_resolution_m <= 0.0) {
    return false;
  }
  if (!std::isfinite(config.hardware.peak_power_w) || config.hardware.peak_power_w <= 0.0 ||
      !std::isfinite(config.hardware.antenna_gain_db) ||
      !std::isfinite(config.hardware.receiver_noise_figure_db) ||
      config.hardware.receiver_noise_figure_db < 0.0 ||
      !std::isfinite(config.hardware.system_loss_db) || config.hardware.system_loss_db < 0.0) {
    return false;
  }
  return true;
}

}  // namespace session
}  // namespace sar
