#include "airborne_radar/decision/classifier/ThreatAssessmentEvaluator.h"

#include <cmath>

#include <spdlog/spdlog.h>

namespace airborne_radar {
namespace decision {
namespace classifier {

namespace {

/**
 * @brief 仓储匹配结果通过验收所需的最小概率阈值。
 * @note 代码行为依据：仅当匹配概率不低于该阈值时，仓储结果才可能覆盖启发式分类。
 */
const float kMinRepositoryMatchProbability = 0.55f;

/**
 * @brief 仓储匹配结果通过验收所允许的最大距离阈值。
 * @note 代码行为依据：仅当匹配距离不高于该阈值时，仓储结果才会被视为可接受匹配。
 */
const float kMaxRepositoryMatchDistance = 1.80f;

/**
 * @brief 触发高威胁控制路径所需的最小置信度阈值。
 * @note 代码行为依据：该值位于单次 tentative 命中的 `0.35` 与 confirmed / 持续命中的 `0.70` 之间，用于区分低证据与可触发激进控制的目标。
 */
const float kHighThreatConfidenceThreshold = 0.55f;

/**
 * @brief 向分类结果列表追加单个目标类型标签。
 * @param[in,out] categories 待写入的分类结果列表。
 * @param target_type 待追加的目标类型标签。
 */
void AppendType(common::TargetCategoryList* categories,
                const std::string& target_type) {
  if (categories != nullptr) {
    categories->emplace_back(target_type);
  }
}

} // namespace

ThreatAssessmentEvaluator::ThreatAssessmentEvaluator(
    const environment::database::IFeatureRepository* feature_repository)
    : feature_repository_(feature_repository) {}

void ThreatAssessmentEvaluator::Evaluate(
    const common::DecisionInputFrame& input_frame, pipeline::TacticalStateStore& state_store,
    pipeline::TacticalEvaluationState& evaluation_state) const {
  evaluation_state.target_classification_result.clear();
  evaluation_state.target_classification_result.reserve(input_frame.tracks.size());
  evaluation_state.lpi_source_info.has_recon_platform = false;

  if (input_frame.tracks.empty()) {
    AppendType(&evaluation_state.target_classification_result, "UNKNOWN");
    spdlog::warn(
        "[ThreatAssessmentEvaluator] Empty track snapshot list, classification reset.");
    return;
  }

  for (std::size_t i = 0; i < input_frame.tracks.size(); ++i) {
    const common::DecisionTrackSnapshot& track_snapshot = input_frame.tracks[i];
    const std::string classification = IdentifyTarget(track_snapshot);
    AppendType(&evaluation_state.target_classification_result, classification);
    UpdateLpiSourceInfo(&evaluation_state.lpi_source_info, classification);

    const std::uint64_t track_key = track_snapshot.state.association_key;
    const float previous_confidence =
        state_store.confidence_memory.count(track_key) != 0U
            ? state_store.confidence_memory[track_key]
            : 0.0f;
    const float confidence = UpdateConfidence(track_snapshot, previous_confidence);
    state_store.confidence_memory[track_key] = confidence;
    state_store.threat_memory[track_key] = ComputeThreatScore(track_snapshot);

    const bool can_trigger_aggressive_controls =
        track_snapshot.state.status != common::DecisionTrackStatus::kLost &&
        confidence >= kHighThreatConfidenceThreshold &&
        (track_snapshot.evidence.has_measurement_evidence ||
         track_snapshot.state.status == common::DecisionTrackStatus::kConfirmed);

    if (IsHighThreatCategory(classification) && can_trigger_aggressive_controls) {
      evaluation_state.should_reduce_power = true;
    }
    if (track_snapshot.state.jamming_detected && can_trigger_aggressive_controls) {
      evaluation_state.eccm_source_info.has_jamming_signal = true;
    }

    spdlog::info("[ThreatAssessmentEvaluator] Target[{}] -> Classification: {}",
                 i, classification);
  }
}

std::string ThreatAssessmentEvaluator::IdentifyTarget(
    const common::DecisionTrackSnapshot& track_snapshot) const {
  if (feature_repository_ != nullptr) {
    environment::database::FeatureVector input;
    input.Set("speed", track_snapshot.state.speed);
    input.Set("rcs", track_snapshot.state.rcs);
    input.Set("jamming", track_snapshot.state.jamming_detected ? 1.0f : 0.0f);

    environment::database::MatchResult match_result;
    if (feature_repository_->QueryBestMatch(input, match_result)) {
      if (ShouldAcceptRepositoryMatch(match_result)) {
        return match_result.target_type;
      }
      spdlog::debug(
          "[ThreatAssessmentEvaluator] Repository match filtered out (type: {}, "
          "probability: {:.3f}, distance: {:.3f}).",
          match_result.target_type, match_result.probability,
          match_result.distance);
    }
  }

  const float threat_score = ComputeThreatScore(track_snapshot);
  if (threat_score >= 2.0f) {
    return "HIGH_THREAT_TARGET";
  }
  if (threat_score >= 0.8f) {
    return "LOW_THREAT_TARGET";
  }
  return "UNKNOWN";
}

float ThreatAssessmentEvaluator::ComputeThreatScore(
    const common::DecisionTrackSnapshot& track_snapshot) const {
  float threat_score = 0.0f;
  const float track_speed = track_snapshot.state.speed;
  const float track_rcs = track_snapshot.state.rcs;
  const bool jamming_detected = track_snapshot.state.jamming_detected;

  if (track_speed > 300.0f) {
    threat_score += 2.0f;
  } else if (track_speed > 120.0f) {
    threat_score += 1.0f;
  }

  if (track_rcs > 3.0f) {
    threat_score += 1.0f;
  } else if (track_rcs > 1.2f) {
    threat_score += 0.5f;
  }

  if (jamming_detected) {
    threat_score += 1.0f;
  }

  if (track_snapshot.state.status == common::DecisionTrackStatus::kConfirmed) {
    threat_score += 0.25f;
  }

  return threat_score;
}

void ThreatAssessmentEvaluator::UpdateLpiSourceInfo(
    common::LpiSourceInfo* source_info, const std::string& classification) const {
  if (source_info == nullptr || source_info->has_recon_platform) {
    return;
  }

  if (IsHighThreatCategory(classification)) {
    source_info->has_recon_platform = true;
  }
}

bool ThreatAssessmentEvaluator::ShouldAcceptRepositoryMatch(
    const environment::database::MatchResult& match_result) const {
  if (match_result.target_type == "UNKNOWN") {
    return false;
  }
  if (!std::isfinite(match_result.probability) ||
      !std::isfinite(match_result.distance)) {
    return false;
  }
  return match_result.probability >= kMinRepositoryMatchProbability &&
         match_result.distance <= kMaxRepositoryMatchDistance;
}

float ThreatAssessmentEvaluator::UpdateConfidence(
    const common::DecisionTrackSnapshot& track_snapshot,
    float previous_confidence) const {
  float confidence = previous_confidence;
  if (track_snapshot.evidence.has_measurement_evidence) {
    confidence = std::min(1.0f, confidence + 0.35f);
  } else {
    confidence *= 0.60f;
  }

  if (track_snapshot.state.status == common::DecisionTrackStatus::kConfirmed) {
    confidence = std::max(confidence, 0.70f);
  }

  if (track_snapshot.state.status == common::DecisionTrackStatus::kTentative &&
      !track_snapshot.evidence.has_measurement_evidence) {
    confidence = std::min(confidence, 0.30f);
  }

  if (track_snapshot.state.status == common::DecisionTrackStatus::kLost) {
    confidence *= 0.50f;
  }

  return confidence;
}

bool ThreatAssessmentEvaluator::IsHighThreatCategory(
    const std::string& category) const {
  return category == "HIGH_THREAT_TARGET" ||
         category == "HIGH_THREAT_FIGHTER";
}

} // namespace classifier
} // namespace decision
} // namespace airborne_radar
