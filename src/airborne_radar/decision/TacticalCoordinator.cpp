#include "airborne_radar/decision/TacticalCoordinator.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "common/logging/ProjectLog.h"

namespace airborne_radar {
namespace decision {

namespace {

// ===== 探测压力判定阈值 =====
constexpr float kMeaningfulDetectionPressureThreshold = 0.35f;

// ===== 状态存储容量预警阈值 =====
constexpr std::size_t kMaxStateStoreEntries = 2048U;

/**
 * @brief 判断探测质量是否存在可观测压力。
 */
bool HasMeaningfulDetectionPressure(const session::PerceptionQualityInfo& perception_quality_info) {
  return perception_quality_info.input_target_count > 0U &&
         perception_quality_info.detection_stress >= kMeaningfulDetectionPressureThreshold;
}

}  // namespace

// ===== 构造函数 =====

TacticalCoordinator::TacticalCoordinator(const environment::IFeatureRepository* feature_repository)
    : threat_assessment_evaluator_(feature_repository) {}

// ===== 私有静态方法 =====

void TacticalCoordinator::PruneInactiveTrackState(const session::TrackStateSnapshotList& tracks,
                                                  session::TacticalStateStore* state_store) {
  if (state_store == nullptr) {
    return;
  }

  std::unordered_set<std::uint64_t> active_keys;
  active_keys.reserve(tracks.size());
  for (std::size_t i = 0; i < tracks.size(); ++i) {
    active_keys.insert(tracks[i].association_key);
  }

  if (state_store->confidence_memory.size() > kMaxStateStoreEntries) {
    // 中译：置信度记忆条目数（{}）超过上限（{}），可能表明航迹 ID 未被正确回收。
    // 标识：状态存储容量预警——条目持续增长说明失配航迹未清理，
    //       长期运行可能导致内存与性能劣化。
    PROJECT_LOG_WARN(
        "[TacticalCoordinator] TacticalStateStore confidence_memory size ({}) exceeds limit ({}). "
        "This may indicate track IDs are not being properly recycled.",
        state_store->confidence_memory.size(), kMaxStateStoreEntries);
  }

  for (auto it = state_store->confidence_memory.begin();
       it != state_store->confidence_memory.end();) {
    if (active_keys.count(it->first) == 0U) {
      it = state_store->confidence_memory.erase(it);
    } else {
      ++it;
    }
  }

  for (auto it = state_store->threat_memory.begin(); it != state_store->threat_memory.end();) {
    if (active_keys.count(it->first) == 0U) {
      it = state_store->threat_memory.erase(it);
    } else {
      ++it;
    }
  }
}

std::string TacticalCoordinator::BuildDecisionSummary(
    const session::DecisionInputFrame& input_frame,
    const ThreatAssessmentEvaluator::Result& threat_result, const LpiEvaluator::Result& lpi_result,
    const EccmEvaluator::Result& eccm_result) {
  (void)threat_result;
  std::vector<std::string> causes;
  if (!input_frame.interference_observations.empty()) {
    causes.push_back("receiver-rf-observation");
  }
  if (HasMeaningfulDetectionPressure(input_frame.perception_quality_info)) {
    causes.push_back("detection-pressure");
  }

  std::string summary;
  if (eccm_result.eccm_activated) {
    summary = "protected-emission";
  } else if (lpi_result.requests_power_reduction) {
    summary = "threat-response";
  } else {
    summary = "baseline";
  }

  if (causes.empty()) {
    return summary;
  }

  summary += "(";
  for (std::size_t i = 0; i < causes.size(); ++i) {
    if (i != 0U) {
      summary += "+";
    }
    summary += causes[i];
  }
  summary += ")";
  return summary;
}

// ===== Evaluate（主入口）=====

session::TacticalDecisionResult TacticalCoordinator::Evaluate(
    const session::DecisionInputFrame& input_frame, session::TacticalStateStore& state_store) {
  session::TacticalDecisionResult result;

  // ==== [1] 确定 ECCM 触发信号 ====
  const bool has_receiver_rf_observation = !input_frame.interference_observations.empty();

  // ==== [2] 威胁评估 ====
  const ThreatAssessmentEvaluator::Result threat_result =
      threat_assessment_evaluator_.Evaluate(input_frame, state_store);

  // ==== [3] LPI 发射控制 ====
  std::vector<session::TacticalProposal> all_proposals;
  LpiEvaluator::Result lpi_result;
  lpi_result = lpi_evaluator_.Evaluate(threat_result.lpi_source_info, &all_proposals);

  // ==== [4] ECCM 抗干扰 ====
  EccmEvaluator::Result eccm_result;
  if (has_receiver_rf_observation) {
    eccm_result =
        eccm_evaluator_.Evaluate(input_frame.interference_observations, &all_proposals);
    // 中译：接收机射频观测通过了 J/N 门限，追加 ECCM 反制提案。
    // 标识：干扰态势识别——存在有效干扰观测时进入抗干扰决策分支。
    PROJECT_LOG_DEBUG(
        "[TacticalCoordinator] Receiver RF observation passed J/N gate. Appending ECCM proposals.");
  } else {
    // 中译：环境干净，继续常规作战流程。
    // 标识：无干扰观测时的正常分支——不追加 ECCM 提案。
    PROJECT_LOG_DEBUG("[TacticalCoordinator] Environment is clear. Continuing nominal operation.");
  }

  // ==== [5] 状态清理 ====
  PruneInactiveTrackState(input_frame.tracks, &state_store);

  // ==== [6] 组装结果 ====
  result.target_classification_result.reserve(threat_result.target_classification_result.size());
  for (std::size_t i = 0; i < threat_result.target_classification_result.size(); ++i) {
    result.target_classification_result.push_back(
        session::TargetCategory(threat_result.target_classification_result[i].target_type));
  }
  result.proposals = all_proposals;

  if (eccm_result.eccm_activated) {
    result.selected_mode = session::TacticalMode::kProtectedEmission;
  } else if (lpi_result.requests_power_reduction) {
    result.selected_mode = session::TacticalMode::kThreatResponse;
  } else {
    result.selected_mode = session::TacticalMode::kBaseline;
  }

  state_store.current_mode = result.selected_mode;
  state_store.last_classification_labels.clear();
  state_store.last_classification_labels.reserve(result.target_classification_result.size());
  for (std::size_t i = 0; i < result.target_classification_result.size(); ++i) {
    state_store.last_classification_labels.push_back(
        result.target_classification_result[i].target_type);
  }
  state_store.last_decision_summary =
      BuildDecisionSummary(input_frame, threat_result, lpi_result, eccm_result);

  // 中译：周期决策摘要（周期号、航迹数、选中模式、提案数、关联压力）。
  // 标识：战术决策每周期概况——模式选择与提案数量，供核对决策链路。
  PROJECT_LOG_DEBUG(
      "[TacticalCoordinator] cycle_index={} tracks={} mode={} proposals={} assoc_stress={:.3f}",
      input_frame.cycle_index, input_frame.tracks.size(), static_cast<int>(result.selected_mode),
      result.proposals.size(), input_frame.association_quality_info.association_stress);

  return result;
}

}  // namespace decision
}  // namespace airborne_radar
