#include "airborne_radar/decision/ThreatAssessmentEvaluator.h"

#include <algorithm>
#include <cmath>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace decision {

namespace {

/**
 * @brief 仓储匹配结果通过验收所需的最小概率阈值。
 */
const float kMinRepositoryMatchProbability = 0.55f;

/**
 * @brief 仓储匹配结果通过验收所允许的最大距离阈值。
 */
const float kMaxRepositoryMatchDistance = 1.80f;

}  // namespace

ThreatAssessmentEvaluator::ThreatAssessmentEvaluator(
    const environment::IFeatureRepository* feature_repository)
    : feature_repository_(feature_repository) {}

ThreatAssessmentEvaluator::Result ThreatAssessmentEvaluator::Evaluate(
    const session::DecisionInputFrame& input_frame,
    session::TacticalStateStore& state_store) const {
  Result result;
  result.target_classification_result.reserve(input_frame.tracks.size());

  // 最近威胁追踪：找到高威胁目标中距离最近的
  float nearest_threat_range_km = 1e6f;

  for (std::size_t i = 0; i < input_frame.tracks.size(); ++i) {
    const session::TrackStateSnapshot& track_snapshot = input_frame.tracks[i];
    const session::TargetCategory category = IdentifyTarget(track_snapshot);
    result.target_classification_result.push_back(category);

    // 更新 LPI 来源信息
    UpdateLpiSourceInfo(&result.lpi_source_info, track_snapshot, category.target_type);

    const std::uint64_t track_key = track_snapshot.association_key;
    const float previous_confidence = state_store.confidence_memory.count(track_key) != 0U
                                          ? state_store.confidence_memory[track_key]
                                          : 0.0f;
    const float confidence = UpdateConfidence(track_snapshot, previous_confidence);
    state_store.confidence_memory[track_key] = confidence;
    state_store.threat_memory[track_key] = ComputeThreatScore(track_snapshot);

    // 追踪最近的高威胁目标距离
    if (IsHighThreatCategory(category.target_type)) {
      const float range_m = std::sqrt(track_snapshot.position_x * track_snapshot.position_x +
                                      track_snapshot.position_y * track_snapshot.position_y +
                                      track_snapshot.position_z * track_snapshot.position_z);
      const float range_km = range_m / 1000.0f;
      if (range_km < nearest_threat_range_km) {
        nearest_threat_range_km = range_km;
        result.lpi_source_info.threat_range_km = range_km;
        result.lpi_source_info.threat_closure_speed_mps =
            track_snapshot.speed;  // 速度模长作为接近速率的近似
        result.lpi_source_info.threat_rcs = track_snapshot.rcs;
        // 方位角/俯仰角预留
        result.lpi_source_info.threat_azimuth_deg = 0.0f;
        result.lpi_source_info.threat_elevation_deg = 0.0f;
      }
    }

    // 中译：目标[{}] 分类结果：{}。
    // 标识：逐目标威胁分类摘要——每个航迹的分类标签，供核对分类器输出。
    PROJECT_LOG_DEBUG("[ThreatAssessmentEvaluator] Target[{}] -> Classification: {}", i,
                      category.target_type);
  }

  if (input_frame.tracks.empty()) {
    // 中译：航迹快照列表为空，分类结果已重置。
    // 标识：无航迹周期的正常分支——威胁评估退化为空结果。
    PROJECT_LOG_DEBUG(
        "[ThreatAssessmentEvaluator] Empty track snapshot list, classification reset.");
  }

  return result;
}

session::TargetCategory ThreatAssessmentEvaluator::IdentifyTarget(
    const session::TrackStateSnapshot& track_snapshot) const {
  if (feature_repository_ != nullptr) {
    environment::FeatureVector input;
    input.Set("speed", track_snapshot.speed);
    input.Set("rcs", track_snapshot.rcs);

    environment::MatchResult match_result;
    if (feature_repository_->QueryBestMatch(input, match_result)) {
      if (ShouldAcceptRepositoryMatch(match_result)) {
        session::TargetCategory result(match_result.target_type);
        result.probability = match_result.probability;
        return result;
      }
      // 中译：特征库匹配结果被过滤（类型、概率、距离）。
      // 标识：知识库匹配的置信度门——匹配概率/距离不达标时回退到统计分类，
      //       防止低置信匹配直接决定目标类型。
      PROJECT_LOG_DEBUG(
          "[ThreatAssessmentEvaluator] Repository match filtered out (type: {}, "
          "probability: {:.3f}, distance: {:.3f}).",
          match_result.target_type, match_result.probability, match_result.distance);
    }
  }

  const float threat_score = ComputeThreatScore(track_snapshot);
  if (threat_score >= 2.0f) {
    return session::TargetCategory("HIGH_THREAT_FIGHTER");
  }
  if (threat_score >= 0.8f) {
    return session::TargetCategory("LOW_THREAT_TARGET");
  }
  return session::TargetCategory("UNKNOWN");
}

float ThreatAssessmentEvaluator::ComputeThreatScore(
    const session::TrackStateSnapshot& track_snapshot) const {
  float threat_score = 0.0f;
  const float track_speed = track_snapshot.speed;
  const float track_rcs = track_snapshot.rcs;

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

  if (track_snapshot.status == session::TrackStatus::kConfirmed) {
    threat_score += 0.25f;
  }

  return threat_score;
}

void ThreatAssessmentEvaluator::UpdateLpiSourceInfo(
    model::LpiSourceInfo* source_info, const session::TrackStateSnapshot& track_snapshot,
    const std::string& classification) const {
  if (source_info == nullptr) {
    return;
  }

  if (IsHighThreatCategory(classification) &&
      track_snapshot.status == session::TrackStatus::kConfirmed) {
    source_info->has_recon_platform = true;
  }
}

bool ThreatAssessmentEvaluator::ShouldAcceptRepositoryMatch(
    const environment::MatchResult& match_result) const {
  if (match_result.target_type == "UNKNOWN") {
    return false;
  }
  if (!std::isfinite(match_result.probability) || !std::isfinite(match_result.distance)) {
    return false;
  }
  return match_result.probability >= kMinRepositoryMatchProbability &&
         match_result.distance <= kMaxRepositoryMatchDistance;
}

float ThreatAssessmentEvaluator::UpdateConfidence(const session::TrackStateSnapshot& track_snapshot,
                                                  float previous_confidence) const {
  if (track_snapshot.status == session::TrackStatus::kConfirmed) {
    return std::max(0.70f, std::min(1.0f, previous_confidence + 0.20f));
  }
  if (track_snapshot.status == session::TrackStatus::kTentative) {
    return std::min(0.45f, std::max(0.20f, previous_confidence + 0.10f));
  }
  return previous_confidence * 0.50f;
}

bool ThreatAssessmentEvaluator::IsHighThreatCategory(const std::string& category) const {
  return category == "HIGH_THREAT_FIGHTER";
}

}  // namespace decision
}  // namespace airborne_radar
