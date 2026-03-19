/**
 * @file TacticalCoordinator.h
 * @brief 定义基于协调器模型的默认决策引擎。
 */

#ifndef AIRBORNE_RADAR_DECISION_TACTICAL_COORDINATOR_H_
#define AIRBORNE_RADAR_DECISION_TACTICAL_COORDINATOR_H_

#include "airborne_radar/decision/classifier/ThreatAssessmentEvaluator.h"
#include "airborne_radar/decision/eccm/SurvivabilityEvaluator.h"
#include "1q/airborne_radar/decision/pipeline/ITacticalDecisionEngine.h"
#include "airborne_radar/decision/pipeline/TacticalEvaluation.h"
#include "airborne_radar/decision/lpi/EmissionControlEvaluator.h"
#include "airborne_radar/environment/database/IFeatureRepository.h"

namespace airborne_radar {
namespace decision {
namespace pipeline {

/**
 * @brief 默认的决策协调器实现。
 */
class TacticalCoordinator final : public ITacticalDecisionEngine {
 public:
  /**
   * @brief 构造函数，可选注入特征仓储。
   * @param feature_repository 供威胁识别使用的特征仓储；可为空。
   */
  explicit TacticalCoordinator(
      const environment::database::IFeatureRepository* feature_repository =
          nullptr);

  /**
   * @brief 评估单周期输入并输出战术建议。
   * @param input_frame 当前周期决策输入帧。
   * @param[in,out] state_store 跨周期战术状态存储。
   * @return 本周期的战术决策结果。
   */
  TacticalDecisionResult Evaluate(
      const common::DecisionInputFrame& input_frame,
      TacticalStateStore& state_store) override;

 private:
  classifier::ThreatAssessmentEvaluator threat_assessment_evaluator_;
  lpi::EmissionControlEvaluator emission_control_evaluator_;
  eccm::SurvivabilityEvaluator survivability_evaluator_;
};

} // namespace pipeline
} // namespace decision
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_DECISION_TACTICAL_COORDINATOR_H_
