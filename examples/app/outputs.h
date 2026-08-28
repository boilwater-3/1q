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
#include <memory>
#include <string>
#include <vector>

#include "1q/coordinate/types.h"
#include "1q/navigation/CoverageArea.h"
#include "1q/navigation/RoutePoint.h"
#include "1q/remote_identification_radar/config/RirSessionConfig.h"
#include "1q/remote_identification_radar/session/RirOutputDebugView.h"
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

  /**
   * @brief RIR 站点几何落盘（一次；仅 RIR 场景调用，惰性创建 rir_site.csv）。
   *
   * 写站点 LLA + 扫描体积（转台朝向 scan_center + 相对可扫描限位 + 绝对俯仰域）
   * + 最大作用距离，供 rir_scan_viewer 画扫描扇区。站点 origin 缓存供
   * RecordRirCycle 做 ENU→LLA。
   */
  void RecordRirSite(const oneq::coordinate::LlaPositionDegM& site_origin,
                     const remote_identification_radar::config::RirSessionConfig& rir_config);

  /**
   * @brief RIR 逐周期逐目标状态落盘（每周期一次；仅 RIR 场景，惰性创建
   * rir_targets.csv）。数据源为组件本周期调试视图快照 LastDebugView()：逐目标
   * 视线方位/俯仰、斜距、航迹状态、识别结论；有航迹时 ENU→LLA 填目标位置。
   * max_detected_slant_range_m 为库上报本周期「实际有效目标最大斜距」
   * （RirCycleResult），逐目标行重复落盘，供查看器区分粗筛门 max_range_m。
   */
  void RecordRirCycle(std::uint32_t cycle, double t_sec,
                      const oneq::coordinate::LlaPositionDegM& site_origin,
                      const remote_identification_radar::session::RirOutputDebugView& view,
                      float max_detected_slant_range_m);

  /** @brief 全部输出流刷盘（结束前调用）。 */
  void Flush();

  /** @brief 平台轨迹总行数（摘要打印与冒烟断言；多机 = 周期数 × 机数）。 */
  std::size_t platform_rows() const { return platform_rows_; }

 private:
  examples::CsvWriter platform_csv_;
  examples::CsvWriter truth_csv_;
  examples::CsvWriter zones_csv_;
  examples::CsvWriter route_csv_;
  // RIR 输出惰性创建：非 RIR 场景不产生 rir_site.csv / rir_targets.csv。
  std::unique_ptr<examples::CsvWriter> rir_site_csv_;
  std::unique_ptr<examples::CsvWriter> rir_targets_csv_;
  std::string output_dir_;
  std::size_t platform_rows_{0U};
};

}  // namespace app
}  // namespace component_attachment

#endif  // EXAMPLES_APP_OUTPUTS_H_
