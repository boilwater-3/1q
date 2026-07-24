/**
 * @file EsrResolutionCellLedger.h
 * @brief 定义 ESR 到达时间、瞬时频率和角度分辨单元账本。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_ESR_RESOLUTION_CELL_LEDGER_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_ESR_RESOLUTION_CELL_LEDGER_H_

#include <cstddef>
#include <vector>

#include "1q/electromagnetics/RfScene.h"

namespace electronic_surveillance_radar {
namespace pipeline {

/** @brief 一条入射链路在接收平台本地坐标系中的到达方向。 */
struct EsrArrivalBearing {
  bool defined{false};
  bool azimuth_observable{true}; /**< 天顶/天底方向为 false，不得发布伪造 AoA。 */
  double azimuth_deg{0.0};
  double elevation_deg{0.0};
};

/** @brief 一个发射源在全部可分辨单元中的候选与干扰累计结果。 */
struct EsrResolutionCellCandidate {
  std::size_t source_index{0U};
  double signal_power_w{0.0};       /**< 候选实际活动时间内的平均信号功率。 */
  double interference_power_w{0.0}; /**< 同一分辨单元内、候选活动时间口径的干扰功率。 */
  double estimated_center_frequency_hz{0.0};
  double active_time_s{0.0};
  std::uint32_t effective_pulse_count{1U}; /**< 去重后的物理脉冲数。 */
};

/** @brief 分辨单元账本的确定性输出。 */
struct EsrResolutionCellLedgerResult {
  std::vector<EsrResolutionCellCandidate> candidates{};
};

/**
 * @brief 按到达时间、瞬时频率和角度构建接收分辨单元账本。
 *
 * @note 输入链路和 bearing 必须按同一稳定 emission identity 顺序排列。
 *       单元内功率最强的外部源成为候选，其余功率只作为该单元的干扰。
 *       同平台源没有可发布 AoA，仅作为调谐通道内的自扰功率；天顶/天底
 *       方位奇点保留在极区单元中作为非候选功率，不得使整帧拒绝。
 */
bool TryBuildEsrResolutionCellLedger(
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& incident_links,
    const std::vector<EsrArrivalBearing>& bearings,
    const oneq::electromagnetics::RfSceneReceiverState& receiver,
    double angular_resolution_deg, EsrResolutionCellLedgerResult* result);

}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_ESR_RESOLUTION_CELL_LEDGER_H_
