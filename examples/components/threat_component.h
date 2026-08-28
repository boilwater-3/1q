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
#include "core/signals.h"

namespace component_attachment {

/**
 * @brief 威胁评估组件：融合态势 + 运动学 → 威胁分/等级。
 *
 * 输入订阅融合态势事件（on_fusion_updated，证据侧）与 AR 航迹状态事件
 * （on_ar_track_state，属性侧），逐目标缓存后组装评估输入——不调用融合/
 * AR 组件方法（集成方同走事件机制）。
 *
 *  * @note 与库内算法面的边界：本组件只做"订阅 + 组装 + 事件化"，评估逻辑全在
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
  void Step(World& world, double dt_sec) override;

 private:
  /// 融合态势事件缓存条目（证据侧：键 → 置信度，周期号新鲜度过滤）。
  struct FusionSnapshot {
    std::uint64_t cycle{0U};
    double confidence{0.0};
  };
  /// AR 航迹状态事件缓存条目（属性侧：速度/RCS/位置，周期号新鲜度过滤）。
  struct ArTrackSnapshot {
    std::uint64_t cycle{0U};
    double speed_m_per_s{0.0};
    double rcs_m2{0.0};
    double position_x_m{0.0};
    double position_y_m{0.0};
    double position_z_m{0.0};
  };

  /// 融合态势事件回调（逐目标缓存）。
  void OnFusionUpdated(const FusionUpdatedEvent& event);
  /// AR 航迹状态事件回调（逐航迹缓存）。
  void OnArTrackState(const ArTrackStateEvent& event);
  /// 信号惰性连接（首次 Step 接线；scoped_connection 随组件析构断开）。
  void EnsureSignalConnections(World& world);

  threat_assessment::ThreatEvaluator evaluator_; /**< 评估器（纯函数式，无跨周期状态） */
  std::unordered_map<std::uint64_t, FusionSnapshot> fusion_by_key_{}; /**< 证据侧缓存 */
  std::unordered_map<std::uint64_t, ArTrackSnapshot> ar_tracks_by_key_{}; /**< 属性侧缓存 */
  boost::signals2::scoped_connection fusion_connection_{}; /**< 融合信号订阅（首次 Step 惰性连接） */
  boost::signals2::scoped_connection ar_track_connection_{}; /**< AR 航迹信号订阅（同上） */
  std::vector<threat_assessment::ThreatResult> results_{};
  std::size_t high_threat_count_{0U};
  /** @brief 上一周期各目标威胁等级（升级判定；示例层跨周期状态）。 */
  std::unordered_map<std::uint64_t, threat_assessment::ThreatLevel> prev_levels_{};
  std::size_t last_level_up_count_{0U}; /**< 本周期升级事件数（视图摘要行用） */

  /// 输入组装：融合态势缓存（证据侧）+ AR 航迹缓存按键补充属性侧（速度/距离/RCS）。
  std::vector<threat_assessment::ThreatEvaluationInput> BuildEvaluationInputs(
      std::uint64_t cycle) const;
  /// 等级升级判定（首见按低威胁计）+ 威胁更新事件发布（升级沿写关键事件）。
  void PublishThreatEvents(World& world);
  /// 每周期威胁态势视图摘要行（等级分布 + 最高威胁目标）。
  void LogThreatView(World& world);
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_THREAT_COMPONENT_H_
