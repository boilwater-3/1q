/**
 * @file demo_output.h
 * @brief 自定义实体-组件示例：统一契约 v2 可视化 CSV 落盘与事件消费侧。
 *
 * DemoOutputs —— 与共享查看器 examples/common/viz/build_viewer.py 对齐的
 * 多机可视化落盘：platform_track（aircraft_id 列）/ target_truth（entity_type
 * 列）/ route_plan（aircraft_id 列）/ zones.csv（巡逻区域）；DecisionListener
 * —— 高置信威胁事件链演示（Fusion → decision → command）。集成端日志
 * （integration_events.log 事件行 + integration_views.log 各组件每周期调试
 * 视图人读摘要行）与库日志（1q_library.log）由 logger/logger.h 承担
 * （组件源文件内日志宏 + 视图摘要直写，字符串归属组件）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_DEMO_OUTPUT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_DEMO_OUTPUT_H_

#include <cstddef>
#include <cstdint>
#include <string>

#include "1q/navigation/CoverageArea.h"
#include "1q/navigation/RoutePoint.h"
#include "csv_writer.h"
#include "core/world.h"
#include "components/ar_sensor_component.h"
#include "components/flight_component.h"
#include "components/rir_sensor_component.h"
#include "scene_script.h"

namespace component_attachment {
namespace demo {

/**
 * @brief 统一契约 v2 可视化 CSV 落盘（共享查看器消费）。
 *
 * 每周期由主程序调 RecordPlatformRow / RecordTruthRow 落盘；航路与区域在
 * 装配后写一次（RecordRoute / RecordZones）。行数计数供结束摘要与冒烟断言
 * 读取。构造打开全部输出文件（CsvWriter 打开失败即 abort，与既有 CSV
 * 语义一致）。
 */
class DemoOutputs {
 public:
  explicit DemoOutputs(const std::string& output_dir);
  ~DemoOutputs() = default;

  DemoOutputs(const DemoOutputs&) = delete;
  DemoOutputs& operator=(const DemoOutputs&) = delete;

  /** @brief 飞行器轨迹行落盘（周期级；aircraft_id 区分各机，model 列 = jsbsim/kinematic）。 */
  void RecordPlatformRow(std::uint32_t cycle, double t_sec, std::uint32_t aircraft_id,
                         const FlightComponent& flight);

  /** @brief 目标真值行落盘（周期级；ECEF → LLA，entity_type 透出空中/地面）。 */
  void RecordTruthRow(std::uint32_t cycle, double t_sec, const TargetEcefState& target);

  /** @brief 巡逻区域落盘（一次；polygon 每顶点一行 / circle 一行 + 半径）。 */
  void RecordZones(const std::string& name, const navigation::CoverageArea& area);

  /** @brief 飞行器航路落盘（一次；aircraft_id 归属各机）。 */
  void RecordRoute(std::uint32_t aircraft_id,
                   const std::vector<navigation::RoutePoint>& route);

  /** @brief 全部输出流刷盘（结束前调用）。 */
  void Flush();

  /** @brief 平台轨迹总行数（摘要打印与冒烟断言；多机 = 周期数 × 机数）。 */
  std::size_t platform_rows() const { return platform_rows_; }

 private:
  examples::CsvWriter platform_csv_;
  examples::CsvWriter truth_csv_;
  examples::CsvWriter zones_csv_;
  examples::CsvWriter route_csv_;
  std::size_t platform_rows_{0U};
};

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
  World& world_;
  double high_threat_confidence_{3.0};
  bool issued_{false};
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
 *  - kEnableAntiFalseTarget：纯日志演示（无库接口，路由器不执行）。
 *
 * 键解析（融合键 → 外部目标 ID）：融合键直挂场景目标 ID（RIR 通道/场景脚本
 * 指令）直接用；否则为 AR 内部 association_key，经 AR 组件最近成功周期的
 * 航迹归属表（last_track_attributions）翻译；均失败则记 command_dropped
 * 事件跳过（守已知边界，不做兜底猜测）。
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

  /** @brief 已执行指令数（下发补丁的；结束摘要用）。 */
  std::uint32_t executed_count() const { return executed_; }

  /** @brief 未执行指令数（键无法解析或无传感器可下发）。 */
  std::uint32_t dropped_count() const { return dropped_; }

 private:
  void OnCommand(const CommandIssuedEvent& event);
  std::uint64_t ResolveExternalTargetId(std::uint64_t target_key) const;

  World& world_;
  ArSensorComponent* ar_{nullptr};
  RirSensorComponent* rir_{nullptr};
  std::vector<std::uint64_t> scene_target_ids_{};
  boost::signals2::scoped_connection connection_{};
  std::uint32_t executed_{0U};
  std::uint32_t dropped_{0U};
};

}  // namespace demo
}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_DEMO_OUTPUT_H_
