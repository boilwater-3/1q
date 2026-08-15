/**
 * @file RirSessionConfigValidation.cpp
 * @brief 远程识别雷达会话配置校验实现。
 *
 * 校验逻辑为 `ArSessionConfigBuilder.cpp` 识别校验段（审计基线 96de367c）
 * 的平移改写：字段路径改为 `policy.recognition.*` 域内、issue code 改为
 * `rir.validation.recognition_*`（RirIssueCodes.h），语义与门限值不变。
 */

#include "1q/remote_identification_radar/config/RirSessionConfigValidation.h"

#include <cmath>

#include "1q/remote_identification_radar/session/RirIssueCodes.h"

namespace remote_identification_radar {
namespace config {

namespace {

bool IsFinite(float value) { return std::isfinite(value); }

void PushIssue(session::RirIssueList* issues, const char* code, const char* field,
               const std::string& message) {
  session::RirIssue issue;
  issue.severity = session::RirIssueSeverity::kError;
  issue.phase = session::RirIssuePhase::kInputValidation;
  issue.code = code;
  issue.field = field;
  issue.message = message;
  issues->push_back(issue);
}

}  // namespace

session::RirIssueList ValidateRirSessionConfig(const RirSessionConfig& config) {
  session::RirIssueList issues;
  const RirRecognitionPolicy& recognition = config.policy.recognition;
  const RirRecognitionFeatureWeights& weights = recognition.feature_weights;

  // 权重：有限、[0, 1]、四者之和为 1.0。
  const float weight_sum = weights.rcs_weight + weights.motion_weight +
                           weights.polarization_weight + weights.range_profile_weight;
  if (!IsFinite(weights.rcs_weight) || !IsFinite(weights.motion_weight) ||
      !IsFinite(weights.polarization_weight) || !IsFinite(weights.range_profile_weight) ||
      weights.rcs_weight < 0.0f || weights.rcs_weight > 1.0f ||
      weights.motion_weight < 0.0f || weights.motion_weight > 1.0f ||
      weights.polarization_weight < 0.0f || weights.polarization_weight > 1.0f ||
      weights.range_profile_weight < 0.0f || weights.range_profile_weight > 1.0f ||
      std::fabs(weight_sum - 1.0f) > 1.0e-5f) {
    PushIssue(&issues, session::codes::kRecognitionWeightsInvalid,
              "policy.recognition.feature_weights",
              "Recognition feature weights must be finite values in [0, 1] summing to 1.0.");
  }

  // 数据库路径：启用识别时必须非空。
  if (recognition.enabled && recognition.database_path.empty()) {
    PushIssue(&issues, session::codes::kRecognitionDatabasePathMissing,
              "policy.recognition.database_path",
              "Recognition database path must be non-empty when recognition is enabled.");
  }

  // 判定门限：[0, 1]。
  if (!IsFinite(recognition.acceptance_score) || recognition.acceptance_score < 0.0f ||
      recognition.acceptance_score > 1.0f || !IsFinite(recognition.minimum_margin) ||
      recognition.minimum_margin < 0.0f || recognition.minimum_margin > 1.0f) {
    PushIssue(&issues, session::codes::kRecognitionThresholdInvalid,
              "policy.recognition.acceptance_score / minimum_margin",
              "Recognition acceptance score and minimum margin must be finite values in [0, 1].");
  }

  // 积累计数：至少为 1。
  if (recognition.min_confirmed_hits == 0U || recognition.min_observation_count == 0U) {
    PushIssue(&issues, session::codes::kRecognitionAccumulationInvalid,
              "policy.recognition.min_confirmed_hits / min_observation_count",
              "Recognition accumulation counts must be at least 1.");
  }

  // 时间范围：保持时间非负；最大距离/驻留/累积窗口有限且为正。
  if (!IsFinite(recognition.result_hold_sec) || recognition.result_hold_sec < 0.0f ||
      !IsFinite(recognition.max_range_m) || recognition.max_range_m <= 0.0f ||
      !IsFinite(recognition.recognition_dwell_sec) || recognition.recognition_dwell_sec <= 0.0f ||
      !IsFinite(recognition.accumulation_window_sec) ||
      recognition.accumulation_window_sec <= 0.0f) {
    PushIssue(&issues, session::codes::kRecognitionTimeRangeInvalid,
              "policy.recognition.result_hold_sec / max_range_m / recognition_dwell_sec / "
              "accumulation_window_sec",
              "Recognition hold time must be non-negative; max range, dwell and accumulation "
              "window must be finite and positive.");
  }

  return issues;
}

}  // namespace config
}  // namespace remote_identification_radar
