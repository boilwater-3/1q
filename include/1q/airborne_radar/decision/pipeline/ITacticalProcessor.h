// Copyright 2026. All Rights Reserved.
//
// Description: 定义基于责任链模式的战术处理流骨架接口。

#ifndef AIRBORNE_RADAR_DECISION_PIPELINE_I_TACTICAL_PROCESSOR_H_
#define AIRBORNE_RADAR_DECISION_PIPELINE_I_TACTICAL_PROCESSOR_H_

#include <memory>
#include <utility>

#include "1q/airborne_radar/core/context/DecisionContext.h"
#include "1q/airborne_radar/core/pipeline/IChainProcessor.h"

namespace airborne_radar {
namespace decision {
namespace pipeline {

/// @brief ITacticalProcessor 是构成行为决策流责任链的节点基础接口。
/// 子类（如 TargetClassifier, LpiController,
/// EccmController）须实现自己的具体策略逻辑。
class ITacticalProcessor
    : public core::pipeline::IChainProcessor<core::context::DecisionContext> {
public:
  virtual ~ITacticalProcessor() {}

  /// @brief 决策层专用封装，保持旧链式调用风格。
  ITacticalProcessor *SetNext(std::unique_ptr<ITacticalProcessor> next) {
    core::pipeline::IChainProcessor<core::context::DecisionContext>
        *next_processor = core::pipeline::IChainProcessor<
            core::context::DecisionContext>::SetNext(std::move(next));
    return static_cast<ITacticalProcessor *>(next_processor);
  }

  /// @brief 兼容入口：驱动整条决策责任链执行。
  void ProcessTactics(core::context::DecisionContext &context) {
    core::pipeline::IChainProcessor<core::context::DecisionContext>::Process(
        context);
  }
};

/// @brief TacticalProcessor 以 DecisionContext 为载体，桥接到模块视图。
/// @tparam TView 该处理器可见的上下文视图类型。
template <typename TView>
class TacticalProcessor
    : public core::pipeline::ChainProcessorWithView<
          core::context::DecisionContext, TView, ITacticalProcessor> {};

} // namespace pipeline
} // namespace decision
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_DECISION_PIPELINE_I_TACTICAL_PROCESSOR_H_
