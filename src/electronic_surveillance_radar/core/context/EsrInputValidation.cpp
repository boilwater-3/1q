#include "1q/electronic_surveillance_radar/core/context/EsrInputValidation.h"

#include <cmath>
#include <cstddef>
#include <string>

namespace electronic_surveillance_radar {
namespace core {
namespace context {

namespace {

/**
 * @brief 生成一条 ESR 输入校验问题。
 * @param[in] severity 问题严重级别。
 * @param[in] code 结构化问题编码。
 * @param[in] emitter_index 辐射源索引；若无特定索引则为 `size_t(-1)`。
 * @param[in] message 面向调用方的短消息。
 * @return 组装后的校验问题。
 */
EsrValidationIssue MakeIssue(EsrValidationSeverity severity,
                             EsrValidationCode code, std::size_t emitter_index,
                             const std::string& message) {
  EsrValidationIssue issue;
  issue.severity = severity;
  issue.code = code;
  issue.emitter_index = emitter_index;
  issue.message = message;
  return issue;
}

/**
 * @brief 判断浮点输入是否有限。
 * @param[in] value 输入标量。
 * @return 当输入为有限数时返回 `true`。
 */
template <typename T>
bool IsFinite(T value) {
  return std::isfinite(value) != 0;
}

/**
 * @brief 校验单个辐射源输入字段。
 * @param[in] emitter 单个辐射源输入。
 * @param[in] emitter_index 辐射源索引。
 * @param[out] issues 校验问题列表。
 */
void ValidateEmitter(const common::EmitterTruthState& emitter,
                     std::size_t emitter_index,
                     EsrValidationIssueList* issues) {
  if (issues == nullptr) {
    return;
  }

  if (emitter.emitter_id.empty()) {
    issues->push_back(MakeIssue(EsrValidationSeverity::kError,
                                EsrValidationCode::kEmptyEmitterId,
                                emitter_index, "emitter id must not be empty"));
  }

  if (!IsFinite(emitter.carrier_hz) || !IsFinite(emitter.bandwidth_hz) ||
      !IsFinite(emitter.tx_power_w) || !IsFinite(emitter.pulse_width_s) ||
      !IsFinite(emitter.pri_s) ||
      !IsFinite(emitter.pose.position_m.x) ||
      !IsFinite(emitter.pose.position_m.y) ||
      !IsFinite(emitter.pose.position_m.z) ||
      !IsFinite(emitter.beam_state.center_az_deg) ||
      !IsFinite(emitter.beam_state.center_el_deg) ||
      !IsFinite(emitter.beam_state.az_beamwidth_deg) ||
      !IsFinite(emitter.beam_state.el_beamwidth_deg)) {
    issues->push_back(
        MakeIssue(EsrValidationSeverity::kError,
                  EsrValidationCode::kNonFiniteEmitterNumericField,
                  emitter_index,
                  "emitter contains non-finite numeric field"));
  }

  if (emitter.carrier_hz <= 0.0) {
    issues->push_back(MakeIssue(EsrValidationSeverity::kError,
                                EsrValidationCode::kInvalidEmitterFrequency,
                                emitter_index,
                                "emitter carrier frequency must be positive"));
  }
  if (emitter.bandwidth_hz <= 0.0) {
    issues->push_back(MakeIssue(EsrValidationSeverity::kError,
                                EsrValidationCode::kInvalidEmitterBandwidth,
                                emitter_index,
                                "emitter bandwidth must be positive"));
  }
  if (emitter.tx_power_w <= 0.0) {
    issues->push_back(MakeIssue(EsrValidationSeverity::kError,
                                EsrValidationCode::kInvalidEmitterPower,
                                emitter_index,
                                "emitter transmit power must be positive"));
  }
}

}  // namespace

EsrValidationIssueList ValidateEsrCycleInput(const EsrCycleInput& input) {
  EsrValidationIssueList issues;

  if (!IsFinite(input.dt_sec)) {
    issues.push_back(
        MakeIssue(EsrValidationSeverity::kError,
                  EsrValidationCode::kNonFiniteCycleDeltaTime,
                  static_cast<std::size_t>(-1),
                  "cycle delta time must be finite"));
  } else if (input.dt_sec <= 0.0f) {
    issues.push_back(
        MakeIssue(EsrValidationSeverity::kError,
                  EsrValidationCode::kInvalidCycleDeltaTime,
                  static_cast<std::size_t>(-1),
                  "cycle delta time must be positive"));
  }

  for (std::size_t i = 0; i < input.scene_emitters.size(); ++i) {
    ValidateEmitter(input.scene_emitters[i], i, &issues);
  }

  return issues;
}

bool HasEsrValidationError(const EsrValidationIssueList& issues) {
  for (std::size_t i = 0; i < issues.size(); ++i) {
    if (issues[i].severity == EsrValidationSeverity::kError) {
      return true;
    }
  }
  return false;
}

}  // namespace context
}  // namespace core
}  // namespace electronic_surveillance_radar
