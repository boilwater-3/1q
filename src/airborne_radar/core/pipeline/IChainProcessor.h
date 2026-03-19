// Copyright 2026. All Rights Reserved.
//
// Description: 定义可复用的责任链处理器基础模板。

#ifndef AIRBORNE_RADAR_CORE_PIPELINE_I_CHAIN_PROCESSOR_H_
#define AIRBORNE_RADAR_CORE_PIPELINE_I_CHAIN_PROCESSOR_H_

#include <memory>
#include <utility>

namespace airborne_radar {
namespace core {
namespace pipeline {

/// @brief IChainProcessor 提供跨层可复用的责任链节点抽象。
/// @tparam Context 管线在节点间传递的上下文类型。
template <typename Context>
class IChainProcessor {
public:
  virtual ~IChainProcessor() {}

  /// @brief 设置当前节点的后继节点。
  IChainProcessor *SetNext(std::unique_ptr<IChainProcessor> next) {
    next_processor_ = std::move(next);
    return next_processor_.get();
  }

  /// @brief 执行责任链：先处理当前节点，再处理后继节点。
  virtual void Process(Context &context) {
    ProcessNode(context);
    if (next_processor_) {
      next_processor_->Process(context);
    }
  }

protected:
  virtual void ProcessNode(Context &context) = 0;

private:
  std::unique_ptr<IChainProcessor> next_processor_;
};

/// @brief ChainProcessorWithView 为“上下文 -> 视图 -> 节点逻辑”提供模板桥接。
/// @tparam Context 管线上下文类型。
/// @tparam View 当前节点可见的上下文视图类型。
/// @tparam ProcessorBase 可选基类，默认使用 IChainProcessor<Context>。
template <typename Context, typename View,
          typename ProcessorBase = IChainProcessor<Context> >
class ChainProcessorWithView : public ProcessorBase {
protected:
  void ProcessNode(Context &context) final {
    View view = CreateView(context);
    ProcessView(view);
  }

  virtual View CreateView(Context &context) const = 0;
  virtual void ProcessView(View &view) = 0;
};

} // namespace pipeline
} // namespace core
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_CORE_PIPELINE_I_CHAIN_PROCESSOR_H_
