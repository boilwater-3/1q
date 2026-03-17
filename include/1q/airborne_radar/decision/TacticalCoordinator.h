// Copyright 2026. All Rights Reserved.
//
// Description: 定义基于协调器模型的默认决策引擎。

#ifndef AIRBORNE_RADAR_DECISION_TACTICAL_COORDINATOR_H_
#define AIRBORNE_RADAR_DECISION_TACTICAL_COORDINATOR_H_

#include "1q/airborne_radar/decision/classifier/ThreatAssessmentEvaluator.h"
#include "1q/airborne_radar/decision/eccm/SurvivabilityEvaluator.h"
#include "1q/airborne_radar/decision/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/decision/lpi/EmissionControlEvaluator.h"
#include "1q/airborne_radar/environment/database/IFeatureRepository.h"

namespace airborne_radar {
namespace decision {

/// @brief TacticalCoordinator 是默认的决策协调器实现。
class TacticalCoordinator final : public ITacticalDecisionEngine {
 public:
  /// @brief 构造函数，可选注入特征仓储。
  explicit TacticalCoordinator(
      const environment::database::IFeatureRepository* feature_repository =
          nullptr);

  /// @brief 评估单周期输入并输出战术建议。
 TacticalDecisionResult Evaluate(
      const common::DecisionInputFrame& input_frame,
      TacticalStateStore& state_store) override;

 private:
  classifier::ThreatAssessmentEvaluator threat_assessment_evaluator_;
  lpi::EmissionControlEvaluator emission_control_evaluator_;
  eccm::SurvivabilityEvaluator survivability_evaluator_;
};

} // namespace decision
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_DECISION_TACTICAL_COORDINATOR_H_
