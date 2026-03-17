// Copyright 2026. All Rights Reserved.
//
// Description: 定义战术责任链中的电子反对抗策略(ECCM)实现节点。

#ifndef AIRBORNE_RADAR_DECISION_ECCM_ECCM_CONTROLLER_H_
#define AIRBORNE_RADAR_DECISION_ECCM_ECCM_CONTROLLER_H_

#include "1q/airborne_radar/decision/pipeline/ITacticalProcessor.h"

namespace airborne_radar {
namespace decision {
namespace eccm {

/// @brief
/// EccmController（电子反对抗控制器）致力于确保在敌对干扰环境中雷达波束依然正常存活。
/// 能够向硬件追加多项反制手段（例如旁瓣对消和频率捷变）。
class EccmController
    : public pipeline::TacticalProcessor<core::context::EccmControllerView> {
protected:
  core::context::EccmControllerView
  CreateView(core::context::DecisionContext &context) const override;

  /// @brief 决策 ECCM 机能在环境干扰下的触发逻辑。
  void ProcessView(core::context::EccmControllerView &view) override;
};

} // namespace eccm
} // namespace decision
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_DECISION_ECCM_ECCM_CONTROLLER_H_
