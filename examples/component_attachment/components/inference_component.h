/**
 * @file inference_component.h
 * @brief 自定义实体-组件示例：目标推演组件。
 *
 * 组件封装 target_inference 模块引擎：每周期读同实体融合组件输出
 * （FusedTarget 运动学估计），按目标键组装推演输入帧，一次
 * TargetInferenceEngine::Infer 输出轨迹预测/发射点/落点/类型概率；
 * 视图摘要每周期直写集成端日志。挂载序在 Fusion 之后（融合先步进）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_INFERENCE_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_INFERENCE_COMPONENT_H_

#include <vector>

#include "1q/target_inference/InferenceResult.h"
#include "1q/target_inference/TargetInferenceConfig.h"
#include "1q/target_inference/TargetInferenceEngine.h"
#include "core/component.h"

namespace component_attachment {

/**
 * @brief 目标推演组件：融合运动学估计 → 轨迹/发射点/类型推演。
 *
 * 引擎为无状态纯函数面（同 threat_assessment 先例），配置经构造注入。
 * 每周期以携带运动学估计的融合目标为主集合（其余目标无推演输入，
 * 跳过并在视图行计数），Infer 后：
 * - 每目标视图摘要行直写集成端日志（[视图:inference]）；
 * - results() 供 demo 消费（确定性，与融合目标过滤后同序）。
 *
 * @note 与库内算法面的边界：本组件只做"组装 + 消费 + 日志化"，推演逻辑
 *       全在 target_inference 模块；组件自身无跨周期状态。
 */
class InferenceComponent : public Component {
 public:
  explicit InferenceComponent(
      const target_inference::TargetInferenceConfig& config =
          target_inference::TargetInferenceConfig{});
  ~InferenceComponent() override = default;

  InferenceComponent(const InferenceComponent&) = delete;
  InferenceComponent& operator=(const InferenceComponent&) = delete;

  const char* Name() const override { return "TargetInference"; }
  void OnAttach(Entity& host) override { host_ = &host; }
  void Step(World& world, double dt_sec) override;

  /** @brief 当前推演结果（与输入航迹同序）。 */
  const std::vector<target_inference::TargetInferenceResult>& results() const {
    return results_;
  }

 private:
  target_inference::TargetInferenceEngine engine_; /**< 推演引擎（纯函数式）。 */
  Entity* host_{nullptr};
  std::vector<target_inference::TargetInferenceResult> results_{};
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_INFERENCE_COMPONENT_H_
