/**
 * @file threat_component.h
 * @brief 自定义实体-组件示例：威胁评估组件。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_THREAT_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_THREAT_COMPONENT_H_

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "1q/threat_assessment/ThreatEvaluator.h"
#include "1q/threat_assessment/ThreatEvaluatorConfig.h"
#include "1q/threat_assessment/ThreatResult.h"
#include "core/component.h"

namespace component_attachment {

/**
 * @brief 威胁评估组件：融合态势 + 运动学 → 威胁分/等级。
 *
 *  * @note 与库内算法面的边界：本组件只做"组装 + 消费 + 事件化"，评估逻辑全在
 *       threat_assessment 模块；跨周期记忆（升级判定）属示例层职责。
 */
class ThreatComponent : public Component {
 public:
  explicit ThreatComponent(const threat_assessment::ThreatEvaluatorConfig& config =
                               threat_assessment::ThreatEvaluatorConfig{});
  ~ThreatComponent() override = default;

  ThreatComponent(const ThreatComponent&) = delete;
  ThreatComponent& operator=(const ThreatComponent&) = delete;

  const char* Name() const override { return "Threat"; }
  void OnAttach(Entity& host) override { host_ = &host; }
  void Step(World& world, double dt_sec) override;

 private:
  threat_assessment::ThreatEvaluator evaluator_; /**< 评估器（纯函数式，无跨周期状态） */
  Entity* host_{nullptr};
  std::vector<threat_assessment::ThreatResult> results_{};
  std::size_t high_threat_count_{0U};
  /** @brief 上一周期各目标威胁等级（升级判定；示例层跨周期状态）。 */
  std::unordered_map<std::uint64_t, threat_assessment::ThreatLevel> prev_levels_{};
  std::size_t last_level_up_count_{0U}; /**< 本周期升级事件数（视图摘要行用） */

  /// 输入组装：融合目标主集合 + AR 调试视图按键补充属性侧（速度/距离/RCS）。
  std::vector<threat_assessment::ThreatEvaluationInput> BuildEvaluationInputs() const;
  /// 等级升级判定（首见按低威胁计）+ 威胁更新事件发布（升级沿写关键事件）。
  void PublishThreatEvents(World& world);
  /// 每周期威胁态势视图摘要行（等级分布 + 最高威胁目标）。
  void LogThreatView(World& world);
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_THREAT_COMPONENT_H_
