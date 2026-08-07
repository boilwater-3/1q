/**
 * @file demo_output.h
 * @brief 自定义实体-组件示例：平台轨迹落盘与事件消费侧。
 *
 * DemoOutputs —— 平台轨迹 CSV 周期落盘；DecisionListener —— 高置信威胁事件链
 * 演示（Fusion → decision → command）。集成端日志（integration_events.log 事件行
 * + integration_views.log 各组件每周期调试视图人读摘要行）与库日志
 * （1q_library.log）由 components/demo_log.h 承担（组件源文件内日志宏 + 视图
 * 摘要直写，字符串归属组件）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_DEMO_OUTPUT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_DEMO_OUTPUT_H_

#include <cstddef>
#include <cstdint>
#include <string>

#include "csv_writer.h"
#include "core/world.h"
#include "components/flight_component.h"

namespace component_attachment {
namespace demo {

/**
 * @brief 平台轨迹 CSV 周期落盘。
 *
 * 每周期由主程序调 RecordPlatformRow 落盘；行数计数供结束摘要与冒烟断言
 * 读取。构造打开输出文件（CsvWriter 打开失败即 abort，与既有 CSV 语义一致）。
 */
class DemoOutputs {
 public:
  explicit DemoOutputs(const std::string& output_dir);
  ~DemoOutputs() = default;

  DemoOutputs(const DemoOutputs&) = delete;
  DemoOutputs& operator=(const DemoOutputs&) = delete;

  /** @brief 平台轨迹行落盘（周期级）。 */
  void RecordPlatformRow(std::uint32_t cycle, double t_sec, const FlightComponent& flight);

  /** @brief 全部输出流刷盘（结束前调用）。 */
  void Flush();

  /** @brief 行计数（摘要打印与冒烟断言）。 */
  std::size_t platform_rows() const { return platform_rows_; }

 private:
  examples::CsvWriter platform_csv_;
  std::size_t platform_rows_{0U};
};

/**
 * @brief 决策监听器：订阅融合更新事件，高置信威胁首次出现时发布指令事件
 * （事件链演示：Fusion → decision → command）。门限由场景数据注入
 * （SceneData::high_threat_confidence）。
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

}  // namespace demo
}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_DEMO_OUTPUT_H_
