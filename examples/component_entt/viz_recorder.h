/**
 * @file viz_recorder.h
 * @brief 行为层演示可视化数据记录器：逐周期把飞行/传感器/融合数据导出为 CSV。
 *
 * 设计目标：消除"示例是黑盒"——demo 原本只打印终端摘要，本记录器把每周期
 * 可观测数据（平台轨迹、目标真值、AR 航迹、EOS 探测、ESR 假设、融合态势、
 * 航路计划、航点完成事件）流式落盘为 8 个 CSV，供
 * examples/common/viz/build_viewer.py 构建交互式 HTML 查看器。
 *
 * 记录器是纯消费方工具（不进库）：只依赖公开会话结果类型与 components.h；
 * 飞行模型标识（jsbsim/kinematic）由调用方在构造时传入。
 */

#ifndef EXAMPLES_BEHAVIOR_LAYER_VIZ_RECORDER_H_
#define EXAMPLES_BEHAVIOR_LAYER_VIZ_RECORDER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "1q/airborne_radar/session/ArCycleResult.h"
#include "1q/coordinate/types.h"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "csv_writer.h"
#include "components.h"

namespace component_entt {

/**
 * @brief 世界真值目标的一行（导出 target_truth.csv 用）。
 * @note ECEF 位置由记录器转换为度制 LLA 落盘；目标 ID 与 RCS 来自消费方真值脚本。
 *       entity_type 区分空中/地面目标（viewer 以不同线型/标记展示）。
 */
struct TruthTargetRow {
  std::uint64_t target_id{0U};       /**< 真值目标 ID */
  std::string entity_type{"air"};    /**< 实体类型（air / ground） */
  oneq::coordinate::EcefPositionM position{}; /**< ECEF 位置（m） */
  float rcs{0.0f};                   /**< 雷达截面积（m²） */
};

/**
 * @brief 航点完成事件的一行（导出 waypoint_events.csv 用）。
 * @note flight_dynamic 类型不泄漏到本头文件：飞行系统把
 * WaypointSequencingEvent 转成该 POD 再交给记录器（仅 FD 启用时有数据）。
 * waypoint_index 为机动队列索引（demo 单次规划且 next_index 从 0 起时与
 * 航路索引一致；重规划场景需消费方自行换算）。
 */
struct WaypointEventRow {
  double t_sec{0.0};                  /**< 完成时刻仿真时间（s） */
  std::size_t waypoint_index{0U};     /**< 机动队列中的航点索引 */
  bool intermediate{false};           /**< 中间航点语义（true）/ 最终航点语义（false） */
  std::string gate{};                 /**< 命中门（within_radius/plane_crossing/fly_past） */
  double distance_m{0.0};             /**< 完成时刻到航点的水平距离（m） */
  double cross_track_m{0.0};          /**< 完成时刻相对航段的侧距（m） */
  double along_track_m{0.0};          /**< 完成时刻沿航迹距离（m） */
  double threshold_m{0.0};            /**< 完成时刻有效到达半径（m） */
};

/**
 * @brief 可视化数据记录器：输出目录 + 一组流式 CSV 写入器。
 *
 * 生命周期：构造时创建输出目录并打开全部 CSV（route_plan.csv 首次
 * RecordRoute 时创建）；每周期 RecordCycle 追加一行；析构自动关闭。
 * 文件打开失败按 CsvWriter 约定 fatal（示例无该数据无法可视化）。
 */
class VizRecorder {
 public:
  /**
   * @brief 构造并打开输出。
   * @param[in] output_dir 输出目录（不存在时创建）。
   * @param[in] flight_model_jsbsim 平台动力学来源：true=JSBSim 真实飞行，
   *            false=运动学回退（写入 platform_track.csv 的 model 列）。
   */
  VizRecorder(const std::string& output_dir, bool flight_model_jsbsim);

  /** @brief 输出目录（供调用方打印产物位置）。 */
  const std::string& output_dir() const { return output_dir_; }

  /**
   * @brief 记录一个周期：平台轨迹 + 目标真值 + 三传感器输出 + 融合态势。
   * @param[in] cycle 周期号（从 1 起）。
   * @param[in] t_sec 仿真时间（s，= cycle × 行为周期时长）。
   * @param[in] fleet 长机编队状态（平台位置/航向/速度）。
   * @param[in] route 航路计划（wp 进度写入 platform_track.csv）。
   * @param[in] situation 融合态势（fused_tracks.csv）。
   * @param[in] ar AR 周期结果（ar_tracks.csv）。
   * @param[in] esr ESR 周期结果（esr_hypotheses.csv）。
   * @param[in] eos EOS 周期结果（eos_detections.csv）。
   * @param[in] truths 世界真值目标（target_truth.csv；ECEF→LLA）。
   */
  void RecordCycle(std::uint32_t cycle, double t_sec, const FleetStatusComponent& fleet,
                   const RoutePlanComponent& route, const FusedSituationComponent& situation,
                   const airborne_radar::session::ArCycleResult& ar,
                   const electronic_surveillance_radar::session::EsrCycleResult& esr,
                   const electro_optical_sensor::session::EosCycleResult& eos,
                   const std::vector<TruthTargetRow>& truths);

  /**
   * @brief 记录航路计划（首次调用时创建 route_plan.csv；重复调用被忽略）。
   * @param[in] route 已规划的航路（version > 0）。
   */
  void RecordRoute(const RoutePlanComponent& route);

  /**
   * @brief 记录一个巡逻区域到 zones.csv（统一契约 v2：多边形每顶点一行 /
   *         圆形一行 + 半径）。同名区域只写一次（幂等去重）。
   * @param[in] name 区域名称（viewer 地图标注）。
   * @param[in] area 覆盖区域（多边形 / 圆形）。
   */
  void RecordZones(const std::string& name, const navigation::CoverageArea& area);

  /**
   * @brief 追加航点完成事件（仅追加自上次调用以来的新事件，按索引去重）。
   * @param[in] events 当前全部已完成事件（飞行系统收集，容量上限 512）。
   */
  void RecordWaypointEvents(const std::vector<WaypointEventRow>& events);

  /** @brief 立即把所有 CSV 刷到磁盘（结束前调用，确保产物完整）。 */
  void Flush();

 private:
  std::string output_dir_;
  bool flight_model_jsbsim_{false};
  bool route_recorded_{false};
  double last_waypoint_event_t_sec_{0.0}; /**< 已写最后一条航点完成事件时刻（去重游标） */
  std::vector<std::string> zones_recorded_{}; /**< 已写区域名（zones.csv 幂等去重） */
  std::unique_ptr<examples::CsvWriter> platform_track_;
  std::unique_ptr<examples::CsvWriter> target_truth_;
  std::unique_ptr<examples::CsvWriter> ar_tracks_;
  std::unique_ptr<examples::CsvWriter> eos_detections_;
  std::unique_ptr<examples::CsvWriter> esr_hypotheses_;
  std::unique_ptr<examples::CsvWriter> fused_tracks_;
  std::unique_ptr<examples::CsvWriter> route_plan_;
  std::unique_ptr<examples::CsvWriter> waypoint_events_;
  std::unique_ptr<examples::CsvWriter> zones_; /**< 巡逻区域（首次 RecordZones 时创建） */
};

}  // namespace component_entt

#endif  // EXAMPLES_BEHAVIOR_LAYER_VIZ_RECORDER_H_
