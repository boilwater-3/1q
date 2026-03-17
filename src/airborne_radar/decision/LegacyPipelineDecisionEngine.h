// Copyright 2026. All Rights Reserved.
//
// Description: 定义旧责任链到新决策引擎接口的适配器。

#ifndef AIRBORNE_RADAR_SRC_DECISION_LEGACY_PIPELINE_DECISION_ENGINE_H_
#define AIRBORNE_RADAR_SRC_DECISION_LEGACY_PIPELINE_DECISION_ENGINE_H_

#include "1q/airborne_radar/decision/ITacticalDecisionEngine.h"
#include "1q/airborne_radar/decision/pipeline/ITacticalProcessor.h"

namespace airborne_radar {
namespace decision {

/// @brief LegacyPipelineDecisionEngine 将旧责任链接口适配为新决策引擎接口。
class LegacyPipelineDecisionEngine final : public ITacticalDecisionEngine {
 public:
  explicit LegacyPipelineDecisionEngine(
      pipeline::ITacticalProcessor& pipeline_head);

  TacticalDecisionResult Evaluate(
      const common::DecisionInputFrame& input_frame,
      TacticalStateStore& state_store) override;

 private:
  pipeline::ITacticalProcessor& pipeline_head_;
};

} // namespace decision
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SRC_DECISION_LEGACY_PIPELINE_DECISION_ENGINE_H_
