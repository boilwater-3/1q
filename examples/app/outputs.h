/**
 * @file outputs.h
 * @brief 统一契约 v2 可视化 CSV 落盘（原 demo_output.h 拆分）。
 *
 * AppOutputs —— 与共享查看器 examples/common/viz/build_viewer.py 对齐的
 * 多机可视化落盘：platform_track（aircraft_id 列）/ target_truth（entity_type
 * 列）/ route_plan（aircraft_id 列）/ zones.csv（巡逻区域）。集成端日志
 * （integration_events.log 事件行 + integration_views.log 各组件每周期调试
 * 视图人读摘要行）与库日志（1q_library.log）由 logger/logger.h 承担
 * （组件源文件内日志宏 + 视图摘要直写，字符串归属组件）。
 */

#ifndef EXAMPLES_APP_OUTPUTS_H_
#define EXAMPLES_APP_OUTPUTS_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "1q/navigation/CoverageArea.h"
#include "1q/navigation/RoutePoint.h"
#include "csv_writer.h"
#include "components/flight_component.h"
#include "scenes/scene_script.h"

namespace component_attachment {
namespace app {

/**
 * @brief 统一契约 v2 可视化 CSV 落盘（共享查看器消费）。
 *
 * 每周期由主程序调 RecordPlatformRow / RecordTruthRow 落盘；航路与区域在
 * 装配后写一次（RecordRoute / RecordZones）。行数计数供结束摘要与冒烟断言
 * 读取。构造打开全部输出文件（CsvWriter 打开失败即 abort，与既有 CSV
 * 语义一致）。
 */
class AppOutputs {
 public:
  explicit AppOutputs(const std::string& output_dir);
  ~AppOutputs() = default;

  AppOutputs(const AppOutputs&) = delete;
  AppOutputs& operator=(const AppOutputs&) = delete;

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

}  // namespace app
}  // namespace component_attachment

#endif  // EXAMPLES_APP_OUTPUTS_H_
