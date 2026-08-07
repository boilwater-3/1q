/**
 * @file demo_output.h
 * @brief 自定义实体-组件示例：输出落盘与事件消费侧。
 *
 * DemoOutputs —— 周期落盘收拢（平台轨迹 CSV + 三通道调试视图三落盘模式
 * JSONL + issues.csv 规则 14 机器消费路径）；DecisionListener —— 高置信
 * 威胁事件链演示（Fusion → decision → command）。事件日志（events.csv）
 * 由组件源文件内日志宏承担（components/demo_log.h，字符串归属组件）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_DEMO_OUTPUT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_DEMO_OUTPUT_H_

#include <cstdint>
#include <fstream>
#include <string>
#include <unordered_map>

#include "csv_writer.h"
#include "core/world.h"
#include "components/ar_sensor_component.h"
#include "components/eos_sensor_component.h"
#include "components/flight_component.h"
#include "components/sbirs_sensor_component.h"

namespace component_attachment {
namespace demo {

/**
 * @brief 周期落盘收拢：平台轨迹 + 三通道调试视图（三落盘模式）+ issues.csv。
 *
 * 每周期由主程序调 RecordPlatformRow / RecordDebugViews 落盘；计数器
 * （行数/AR 增量行数等）供结束摘要与冒烟断言读取。构造打开全部输出文件，
 * 任一 JSONL 打开失败时 valid() 返回 false（主程序据此退出）。
 */
class DemoOutputs {
 public:
  explicit DemoOutputs(const std::string& output_dir);
  ~DemoOutputs() = default;

  DemoOutputs(const DemoOutputs&) = delete;
  DemoOutputs& operator=(const DemoOutputs&) = delete;

  /** @brief 输出文件全部打开成功。 */
  bool valid() const { return valid_; }

  /** @brief 平台轨迹行落盘（周期级）。 */
  void RecordPlatformRow(std::uint32_t cycle, double t_sec, const FlightComponent& flight);

  /**
   * @brief 三通道调试视图落盘（规则 12 三落盘模式 + 规则 14 issues.csv）：
   * SBIRS 全帧、AR 跨周期增量（状态表由本类持有）、EOS 降频。
   */
  void RecordDebugViews(std::uint32_t cycle, const ArSensorComponent& ar,
                        const EosSensorComponent& eos, const SbirsSensorComponent& sbirs);

  /** @brief 全部输出流刷盘（结束前调用）。 */
  void Flush();

  /** @brief 行计数（摘要打印与冒烟断言）。 */
  std::size_t platform_rows() const { return platform_rows_; }
  std::size_t sbirs_debug_rows() const { return sbirs_debug_rows_; }
  std::size_t ar_delta_rows() const { return ar_delta_rows_; }
  std::size_t eos_debug_rows() const { return eos_debug_rows_; }
  std::size_t issues_rows() const { return issues_rows_; }

 private:
  bool valid_{true};
  examples::CsvWriter platform_csv_;
  examples::CsvWriter issues_csv_;
  std::ofstream sbirs_debug_jsonl_;  /**< SBIRS 全帧（每周期一行 JSON） */
  std::ofstream ar_delta_jsonl_;     /**< AR 跨周期增量（仅状态变化行） */
  std::ofstream eos_debug_jsonl_;    /**< EOS 降频（每 10 周期全帧 + 仅 issues） */
  std::unordered_map<std::uint64_t, airborne_radar::session::ArDebugTrackStatus>
      ar_prev_status_{}; /**< AR 增量模式状态表（本类持有，跨周期累计，只增不减） */
  std::size_t platform_rows_{0U};
  std::size_t sbirs_debug_rows_{0U};
  std::size_t ar_delta_rows_{0U};
  std::size_t eos_debug_rows_{0U};
  std::size_t issues_rows_{0U};
};

/**
 * @brief 决策监听器：订阅融合更新事件，高置信威胁首次出现时发布指令事件
 * （事件链演示：Fusion → decision → command）。
 */
class DecisionListener {
 public:
  explicit DecisionListener(World& world);

  bool issued() const { return issued_; }

 private:
  World& world_;
  bool issued_{false};
};

}  // namespace demo
}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_DEMO_OUTPUT_H_
