/**
 * @file fusion_component.h
 * @brief 自定义实体-组件示例：融合组件。
 *
 * 组件封装 fusion 模块引擎：每周期经 host_ 类型化聚合同实体三个传感器
 * 组件的本周期探测记录（挂载序保证传感器先于融合步进），一次
 * FusionEngine::Update 更新态势；新/消失目标计数随 FusionUpdatedEvent
 * 发布（跨周期通知）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_FUSION_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_FUSION_COMPONENT_H_

#include <memory>
#include <vector>

#include "1q/fusion/FusionEngine.h"
#include "1q/fusion/FusedTarget.h"
#include "core/component.h"

namespace component_attachment {

/**
 * @brief 融合组件：多源探测聚合 + 融合态势更新。
 *
 * 引擎经 std::make_unique<FusionEngine>(config) 构造后移动进组件。
 * targets() 为当前融合目标态势（按 key 升序），供 demo 消费。
 *
 * @note 运行时修改：本组件不提供运行时参数修改入口——fusion 模块参数为
 * 会话级不可变（库内无 RuntimeConfigPatch 设计，FusionEngine 仅构造时
 * 接受 FusionConfig）；需要运行时修改须先补库 API（如
 * FusionEngine::TryApplyRuntimeConfig），不在示例层包装。
 */
class FusionComponent : public Component {
 public:
  explicit FusionComponent(std::unique_ptr<fusion::FusionEngine> engine);
  ~FusionComponent() override = default;

  FusionComponent(const FusionComponent&) = delete;
  FusionComponent& operator=(const FusionComponent&) = delete;

  const char* Name() const override { return "Fusion"; }
  void OnAttach(Entity& host) override { host_ = &host; }
  void Step(World& world, double dt_sec) override;

  /** @brief 当前融合目标态势（按 key 升序）。 */
  const std::vector<fusion::FusedTarget>& targets() const { return targets_; }

 private:
  std::unique_ptr<fusion::FusionEngine> engine_;
  Entity* host_{nullptr};
  std::vector<fusion::FusedTarget> targets_{};
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_FUSION_COMPONENT_H_
