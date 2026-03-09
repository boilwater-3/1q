// Copyright 2026. All Rights Reserved.
//
// Description: 定义战术责任链中的低截获概率控制策略(LPI)实现节点。

#ifndef AIRBORNE_RADAR_SRC_DECISION_LPI_LPI_CONTROLLER_H_
#define AIRBORNE_RADAR_SRC_DECISION_LPI_LPI_CONTROLLER_H_

#include "1q/airborne_radar/decision/pipeline/ITacticalProcessor.h"

namespace airborne_radar {
namespace decision {
namespace lpi {

/// @brief LpiController（低截获概率控制器）评估战术上下文，
/// 基于高危飞行器探测分类或信号拦截风险，自适应降低辐射能力。
class LpiController
    : public pipeline::TacticalProcessor<core::context::LpiControllerView> {
protected:
  core::context::LpiControllerView
  CreateView(core::context::DecisionContext &context) const override;

  /// @brief 决策 LPI 是否需启用。
  void ProcessView(core::context::LpiControllerView &view) override;
};

} // namespace lpi
} // namespace decision
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SRC_DECISION_LPI_LPI_CONTROLLER_H_
