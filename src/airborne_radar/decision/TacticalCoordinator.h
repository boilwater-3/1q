/**
 * @file TacticalCoordinator.h
 * @brief 定义基于协调器模型的默认决策引擎。
 */

#ifndef AIRBORNE_RADAR_DECISION_TACTICAL_COORDINATOR_H_
#define AIRBORNE_RADAR_DECISION_TACTICAL_COORDINATOR_H_

#include <vector>

#include "airborne_radar/decision/ControlReducerTypes.h"
#include "1q/airborne_radar/session/DecisionInputFrame.h"
#include "airborne_radar/decision/EccmEvaluator.h"
#include "airborne_radar/decision/LpiEvaluator.h"
#include "airborne_radar/decision/TacticalDecisionTypes.h"
#include "airborne_radar/decision/ThreatAssessmentEvaluator.h"
#include "airborne_radar/environment/IFeatureRepository.h"

namespace airborne_radar {
namespace decision {

/**
 * @brief 默认的决策协调器实现。
 *
 * 编排 ThreatAssessment → LPI → ECCM 的评估流水线，
 * 各 evaluator 通过明确的 Result 结构体传递数据，不共享中间状态。
 */
class TacticalCoordinator final {
 public:
  /**
   * @brief 构造函数。
   * @param[in] feature_repository 供威胁识别使用的特征仓储；可为空。
   */
  explicit TacticalCoordinator(
      const environment::IFeatureRepository* feature_repository = nullptr);

  /**
   * @brief 评估单周期输入并输出战术建议。
   * @param[in] input_frame 当前周期决策输入帧。
   * @param[in,out] state_store 跨周期战术状态存储。
   * @return 本周期的战术决策结果。
   */
  session::TacticalDecisionResult Evaluate(
      const session::DecisionInputFrame& input_frame,
      session::TacticalStateStore& state_store);

 private:
  /**
   * @brief 构建决策摘要字符串。
   */
  static std::string BuildDecisionSummary(
      const session::DecisionInputFrame& input_frame,
      const ThreatAssessmentEvaluator::Result& threat_result,
      const LpiEvaluator::Result& lpi_result,
      const EccmEvaluator::Result& eccm_result);

  /**
   * @brief 清理失效轨迹的状态记忆。
   */
  static void PruneInactiveTrackState(
      const session::TrackStateSnapshotList& tracks,
      session::TacticalStateStore* state_store);

  ThreatAssessmentEvaluator threat_assessment_evaluator_;
  LpiEvaluator lpi_evaluator_;
  EccmEvaluator eccm_evaluator_;
};

}  // namespace decision
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_DECISION_TACTICAL_COORDINATOR_H_
