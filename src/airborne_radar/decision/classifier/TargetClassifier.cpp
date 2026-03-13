// Copyright 2026. All Rights Reserved.

#include "airborne_radar/decision/classifier/TargetClassifier.h"

#include <cmath>

#include "1q/airborne_radar/environment/database/IFeatureRepository.h"
#include <spdlog/spdlog.h>

namespace airborne_radar {
namespace decision {
namespace classifier {

namespace {
/// @brief 仓储匹配结果过滤阈值：最低接受概率。  
const float kMinRepositoryMatchProbability = 0.55f;
/// @brief 仓储匹配结果过滤阈值：最大接受距离（特征空间距离）。 
const float kMaxRepositoryMatchDistance = 1.80f;

void AppendType(common::TargetCategoryList &categories,
                const std::string &target_type) {
  categories.emplace_back(target_type);
}

} // namespace

core::context::TargetClassifierView TargetClassifier::CreateView(
    core::context::DecisionContext &context) const {
  return core::context::CreateTargetClassifierView(context);
}

void TargetClassifier::ProcessView(core::context::TargetClassifierView &view) {
  // 对输入特征列表中的每个目标独立执行识别。
  view.target_classification_result.clear();
  view.lpi_source_info.has_recon_platform = false;

  if (view.targets_features.empty()) {
    AppendType(view.target_classification_result, "UNKNOWN");
    spdlog::warn("[TargetClassifier] Empty feature list, classification reset.");
    return;
  }

  view.target_classification_result.reserve(view.targets_features.size());

  for (std::size_t i = 0; i < view.targets_features.size(); ++i) {
    const common::TargetFeature &target = view.targets_features[i];
    const float track_speed = target.current_track_speed;
    const float track_rcs = target.current_track_rcs;
    const bool jamming_detected = false; // TODO 不一定是全局干扰，这里感觉不应该是干扰

    // 首先尝试使用外部特征仓储进行分类匹配（带过滤）。
    std::string classification = "UNKNOWN";
    bool accepted_repository_match = false;

    if (feature_repository_ != nullptr) {
      // 构建输入特征向量，注意特征名称需与仓储定义一致。
      environment::database::FeatureVector input;
      input.Set("speed", track_speed);
      input.Set("rcs", track_rcs);
      input.Set("jamming", jamming_detected ? 1.0f : 0.0f);

      // 查询仓储并判断结果是否可接受。
      environment::database::MatchResult match_result;
      if (feature_repository_->QueryBestMatch(input, match_result)) {
        if (ShouldAcceptRepositoryMatch(match_result)) {
          classification = match_result.target_type;
          accepted_repository_match = true;
        } else {
          spdlog::debug(
              "[TargetClassifier] Repository match filtered out (type: {}, "
              "probability: {:.3f}, distance: {:.3f}).",
              match_result.target_type, match_result.probability,
              match_result.distance);
        }
      }
    }
    // 如果仓储匹配不可用或不可靠，则退回到规则算法识别。
    if (!accepted_repository_match) {
      classification = IdentifyTarget(target);
    }
    // 将最终分类结果附加到输出列表中。
    AppendType(view.target_classification_result, classification);
    UpdateLpiSourceInfo(view.lpi_source_info, classification);

    // 记录分类决策过程的关键信息，便于后续分析与调优。
    spdlog::info(
        "[TargetClassifier] Target[{}] -> Classification: {}", i, classification);
  }
}

std::string TargetClassifier::IdentifyTarget(
  const common::TargetFeature &target) const {
  const float threat_score = ComputeThreatScore(target);

  if (threat_score >= 2.0f) {
    return "HIGH_THREAT_TARGET";
  }
  if (threat_score >= 0.8f) {
    return "LOW_THREAT_TARGET";
  }

  return "UNKNOWN";
}

float TargetClassifier::ComputeThreatScore(
  const common::TargetFeature &target) const {
  float threat_score = 0.0f;
  const float track_speed = target.current_track_speed;
  const float track_rcs = target.current_track_rcs;
  const bool jamming_detected = false;

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

  return threat_score;
}

void TargetClassifier::UpdateLpiSourceInfo(
    common::LpiSourceInfo &source_info,
    const std::string &classification) const {
  if (source_info.has_recon_platform) {
    return;
  }

  // TODO 当前先按高威胁标签近似映射侦察类目标，后续接入明确侦察类别体系。
  if (classification == "HIGH_THREAT_TARGET" ||
      classification == "HIGH_THREAT_FIGHTER") {
    source_info.has_recon_platform = true;
  }
}

bool TargetClassifier::ShouldAcceptRepositoryMatch(
    const environment::database::MatchResult &match_result) const {
  if (match_result.target_type == "UNKNOWN") {
    return false;
  }

  if (!std::isfinite(match_result.probability) ||
      !std::isfinite(match_result.distance)) {
    return false;
  }

  if (match_result.probability < kMinRepositoryMatchProbability) {
    return false;
  }

  if (match_result.distance > kMaxRepositoryMatchDistance) {
    return false;
  }

  return true;
}

} // namespace classifier
} // namespace decision
} // namespace airborne_radar
