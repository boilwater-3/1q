/**
 * @file signals.h
 * @brief 自定义实体-组件示例：事件信号集合（Boost.Signals2）。
 *
 * 事件机制直接使用 C++ 常见开源事件库 Boost.Signals2（零自定义分发层），
 * 提供跨周期通知/记录的事件通道：组件发布 = 调用信号
 * （world.signals().xxx(evt)）；消费方订阅 = .connect(handler)，返回
 * boost::signals2::scoped_connection 管理生命周期。
 *
 * boost/1.85.0 已在 conanfile _BASE_DEPS（header_only），本文件仅依赖其头文件。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_CORE_SIGNALS_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_CORE_SIGNALS_H_

#include <boost/signals2/signal.hpp>

#include "events.h"

namespace component_attachment {

/**
 * @brief 事件信号集合：World 持有单一实例，组件与消费方经其通信。
 *
 * 信号成员仅组织库对象（无任何自定义分发逻辑）；类型安全多播、
 * 连接生命周期均由 Boost.Signals2 承担。
 */
struct Signals {
  /** @brief 平台状态更新（FlightComponent 每周期推进后发布）。 */
  boost::signals2::signal<void(const PlatformStateEvent&)> on_platform_state;
  /** @brief 航点到达（FlightComponent 航点完成判定）。 */
  boost::signals2::signal<void(const WaypointReachedEvent&)> on_waypoint_reached;
  /** @brief AR 目标首次确认。 */
  boost::signals2::signal<void(const TargetConfirmedEvent&)> on_target_confirmed;
  /** @brief AR 目标失跟。 */
  boost::signals2::signal<void(const TargetLostEvent&)> on_target_lost;
  /** @brief AR 航迹逐周期状态（威胁评估属性侧输入）。 */
  boost::signals2::signal<void(const ArTrackStateEvent&)> on_ar_track_state;
  /** @brief ESR 辐射源假设。 */
  boost::signals2::signal<void(const EmitterHypothesisEvent&)> on_emitter_hypothesis;
  /** @brief ESR 假设集快照（每成功周期全量；ECM sensor-driven 输入）。 */
  boost::signals2::signal<void(const EsrScanUpdatedEvent&)> on_esr_scan_updated;
  /** @brief EOS 探测。 */
  boost::signals2::signal<void(const EosDetectionEvent&)> on_eos_detection;
  /** @brief SBIRS 探测。 */
  boost::signals2::signal<void(const SbirsDetectionEvent&)> on_sbirs_detection;
  /** @brief SAR 图像产品。 */
  boost::signals2::signal<void(const SarProductEvent&)> on_sar_product;
  /** @brief 融合态势更新。 */
  boost::signals2::signal<void(const FusionUpdatedEvent&)> on_fusion_updated;
  /** @brief 威胁评估更新（ThreatComponent 每周期逐目标发布）。 */
  boost::signals2::signal<void(const ThreatUpdatedEvent&)> on_threat_updated;
  /** @brief 决策指令下发（决策侧订阅融合事件后转发）。 */
  boost::signals2::signal<void(const CommandIssuedEvent&)> on_command_issued;
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_CORE_SIGNALS_H_
