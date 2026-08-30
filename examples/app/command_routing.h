/**
 * @file command_routing.h
 * @brief 事件消费侧的决策监听与指令路由（原 demo_output.h 拆分）。
 *
 * DecisionListener —— 高置信威胁事件链演示（Fusion → decision → command）；
 * CommandRouter —— 外部控制的传感器侧执行端（订阅指令事件 → 键解析 →
 * 运行期补丁下发）。
 */

#ifndef EXAMPLES_APP_COMMAND_ROUTING_H_
#define EXAMPLES_APP_COMMAND_ROUTING_H_

#include <cstdint>
#include <vector>

#include "core/world.h"
#include "components/ar_sensor_component.h"
#include "components/rir_sensor_component.h"
#include "scenes/scene_script.h"

namespace component_attachment {
namespace app {

/**
 * @brief 决策监听器：订阅融合更新事件与威胁评估事件，高置信威胁/高威胁等级
 * 首次出现时发布指令事件（事件链演示：Fusion → decision → command 与
 * Fusion → threat → decision → command）。融合门限由场景数据注入
 * （SceneData::high_threat_confidence）；威胁门限 = 威胁等级 HIGH。
 */
class DecisionListener {
 public:
  explicit DecisionListener(World& world, double high_threat_confidence);

  bool issued() const { return issued_; }

 private:
  void OnFusionUpdated(const FusionUpdatedEvent& event);
  void OnThreatUpdated(const ThreatUpdatedEvent& event);
  void EnsureSignalConnections();

  World& world_;
  double high_threat_confidence_{3.0};
  bool issued_{false};
  boost::signals2::scoped_connection fusion_connection_{};
  boost::signals2::scoped_connection threat_connection_{};
};

/**
 * @brief 指令路由器：外部控制的传感器侧执行端（订阅指令事件 → 键解析 →
 * 运行期补丁下发）。
 *
 * 真实系统里"锁定/指定目标"由外部指挥系统在运行中下令；示例以指令事件为
 * 统一入口（决策侧 DecisionListener 的自动闭环与场景指令脚本的定时下发
 * 走同一信号），本路由器翻译目标键后经各组件公有 TryApplyRuntimeConfig
 * 下发补丁——指定/锁定不再是任务开始配置：
 *  - kEngageHighThreat / kDesignateTarget：AR 切 STT + 指定目标（限时窗口），
 *    RIR（若挂载）同步指定识别；
 *  - kClearDesignation：两传感器清除指定，AR 回扫描模式（示例配置基线 kTas）；
 *  - kEnableAntiFalseTarget：纯日志演示（无库接口），路由器以"受理"终态收口
 *    （command_executed 处置=受理），计入执行数。
 *
 * 键解析（融合键 → 外部目标 ID）：融合键直挂场景目标 ID（RIR 通道/场景脚本
 * 指令）直接用；否则为 AR 内部 association_key，经 AR 组件最近成功周期的
 * 航迹归属表（last_track_attributions）翻译；均失败则记 command_dropped
 * 事件跳过（守已知边界，不做兜底猜测）。任意指令必有终态沿：
 * 指令下发数 = 执行数 + 丢弃数。
 */
class CommandRouter {
 public:
  /**
   * @param[in] world 世界（订阅 on_command_issued；组件指针生命周期由实体持有覆盖全场）
   * @param[in] ar AR 组件（场景未挂载时 nullptr）
   * @param[in] rir RIR 站点组件（场景未挂载时 nullptr）
   * @param[in] scene_targets 场景目标脚本（外部目标 ID 直挂判定集）
   */
  CommandRouter(World& world, ArSensorComponent* ar, RirSensorComponent* rir,
                const std::vector<ScriptedTarget>& scene_targets);

  /** @brief 已执行指令数（下发补丁或演示受理的；结束摘要用）。 */
  std::uint32_t executed_count() const { return executed_; }

  /** @brief 未执行指令数（键无法解析或无传感器可下发）。 */
  std::uint32_t dropped_count() const { return dropped_; }

 private:
  void OnCommand(const CommandIssuedEvent& event);
  void EnsureSignalConnections();
  std::uint64_t ResolveExternalTargetId(std::uint64_t target_key) const;

  World& world_;
  ArSensorComponent* ar_{nullptr};
  RirSensorComponent* rir_{nullptr};
  std::vector<std::uint64_t> scene_target_ids_{};
  boost::signals2::scoped_connection connection_{};
  std::uint32_t executed_{0U};
  std::uint32_t dropped_{0U};
};

/**
 * @brief 落点预报分发回执监听器：订阅 on_impact_forecast_published，写集成端
 *        事件日志（验证分发链路可达；接收方 ID 由构造注入）。
 */
class ImpactForecastReceiptListener {
 public:
  ImpactForecastReceiptListener(World& world, std::uint64_t receiver_entity_id);

 private:
  void OnImpactForecastPublished(const ImpactForecastEvent& event);
  void EnsureSignalConnections();

  World& world_;
  std::uint64_t receiver_entity_id_{0U};
  boost::signals2::scoped_connection impact_forecast_connection_{};
};

}  // namespace app
}  // namespace component_attachment

#endif  // EXAMPLES_APP_COMMAND_ROUTING_H_
