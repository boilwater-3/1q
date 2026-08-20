/**
 * @file SarScanBurst.h
 * @brief 扫描模式(ScanSAR)elevation 向 burst 调度序列生成。
 *
 * ScanSAR = 平台直线匀速 + 天线在 elevation(距离)向周期性切换子带。波束调度与平台轨迹
 * 正交分离(契约 scansar_mode.md §3.1,迁移聚束 §3.1 方案 2),在回波生成处汇合。
 *
 * 与聚束 `SarSpotlightBeam`(方位向时变波束)正交:聚束方位扫,ScanSAR 距离扫。
 */

#ifndef ONEQ_SRC_SAR_GEOMETRY_SAR_SCAN_BURST_H_
#define ONEQ_SRC_SAR_GEOMETRY_SAR_SCAN_BURST_H_

#include <cstdint>
#include <vector>

#include "sar/geometry/SarGeometry.h"

namespace sar {
namespace geometry {

/**
 * @brief 单子带定义(elevation 向距离窗口)。
 *
 * 每个子带覆盖一段斜距区间 [near_range_m, far_range_m],聚焦时以其中心为参考斜距。
 */
struct ScanSubswath {
  double near_range_m{0.0};  /**< 子带近端斜距 */
  double far_range_m{0.0};   /**< 子带远端斜距 */
};

/**
 * @brief 单脉冲的 ScanSAR burst 调度状态(与 PlatformPulseState 时间对齐)。
 *
 * 与聚束 `SpotlightBeamState`(单个 boresight_azimuth_rad)对比:ScanSAR 的 burst 状态
 * 需要子带索引 + 距离窗口(burst 本质是距离分段),而非单一角度。
 */
struct ScanBurstState {
  double time_s{0.0};
  std::uint32_t subswath_index{0U};  /**< 当前 burst 指向的子带编号 */
  double near_range_m{0.0};          /**< 该子带近端斜距(elevation 门控用) */
  double far_range_m{0.0};           /**< 该子带远端斜距(elevation 门控用) */
  bool illuminated{false};           /**< 该脉冲是否照射该子带(burst 驻留/间隙标记) */
};

/**
 * @brief ScanSAR burst 调度配置。
 *
 * 调度律(契约 §3.2):每个 burst 周期 T_cycle = N_swath × dwell_time_s 内,天线在 N_swath
 * 个子带间轮流驻留 dwell_time_s。同一子带的相邻 burst 中心方位间距
 * Δx_burst = v × T_cycle,需满足方位无缝隙覆盖(否则门控拒绝)。
 */
struct ScanBurstScheduleConfig {
  std::vector<ScanSubswath> subswaths{};  /**< N_swath 个子带(按轮转顺序) */
  double dwell_time_s{0.0};               /**< 单子带单次驻留时间(burst 长度) */
  std::vector<double> pulse_times_s{};    /**< 与平台脉冲序列对齐的慢时间 */
};

/**
 * @brief 生成 ScanSAR burst 调度序列:周期性子带轮转。
 *
 * 对每个脉冲,按其慢时间 t 落入的 burst 周期片段确定:
 *   cycle_index = floor(t / T_cycle)
 *   slot_index  = floor((t - cycle_index·T_cycle) / dwell_time_s)
 *   subswath_index = slot_index mod N_swath
 *   illuminated = true(脉冲落在该子带的驻留窗口内)
 *
 * 单子带退化不变量:N_swath=1 时,所有脉冲 illuminated=true、subswath_index=0,
 * 等价全程照射单子带(退化为条带)。
 *
 * @param config 调度配置(子带列表 + 驻留时间 + 脉冲慢时间)。
 * @param burst_states 输出 burst 调度序列(与 pulse_times_s 等长,时间对齐)。
 * @return 成功则 true;子带空、驻留时间非正、脉冲空或指针空则 false。
 */
bool GenerateScanBurstSchedule(const ScanBurstScheduleConfig& config,
                               std::vector<ScanBurstState>* burst_states);

/**
 * @brief 子带中心斜距(聚焦参考斜距)。
 *
 * @param subswath 子带。
 * @return (near + far) / 2;near/far 非有限或顺序颠倒则返回 0。
 */
double SubswathCenterRange(const ScanSubswath& subswath);

/**
 * @brief 目标斜距是否落在子带 elevation 窗口内(回波门控用)。
 *
 * 半开区间 [near, far),避免子带边界目标重复计入相邻子带。
 *
 * @param subswath 子带。
 * @param slant_range_m 目标斜距。
 * @return 落在 [near, far) 则 true;否则 false。
 */
bool IsInSubswath(const ScanSubswath& subswath, double slant_range_m);

/**
 * @brief ScanSAR 场景组合配置(平台直线匀速 + 天线 elevation 子带轮转)。
 */
struct ScanSarTrackConfig {
  StraightStripmapTrackConfig platform_track{};  /**< 平台直线匀速轨迹 */
  std::vector<ScanSubswath> subswaths{};          /**< N_swath 个子带 */
  double dwell_time_s{0.0};                       /**< 单子带单次驻留时间 */
};

/**
 * @brief 生成 ScanSAR 场景的平台轨迹 + burst 调度序列。
 *
 * 内部:GenerateStraightStripmapTrack 产平台脉冲 → 抽慢时间 → GenerateScanBurstSchedule
 * 产 burst 调度。两个序列时间对齐、等长。
 *
 * @return 成功则 true;平台轨迹或调度生成失败则 false。
 */
bool GenerateScanSarTrack(const ScanSarTrackConfig& config,
                          std::vector<PlatformPulseState>* platform_pulses,
                          std::vector<ScanBurstState>* burst_states);

}  // namespace geometry
}  // namespace sar

#endif  // ONEQ_SRC_SAR_GEOMETRY_SAR_SCAN_BURST_H_
