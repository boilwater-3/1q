/**
 * @file ArDetectionCellResolver.h
 * @brief 定义 AR 统计级 range-Doppler-beam-time-frequency detection cell 求解器。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_DETECTION_AR_DETECTION_CELL_RESOLVER_H_
#define AIRBORNE_RADAR_SIGNAL_DETECTION_AR_DETECTION_CELL_RESOLVER_H_

#include <vector>

#include "1q/electromagnetics/RfScene.h"

namespace airborne_radar {
namespace signal {
namespace detection {

/** @brief detection cell 所需的冻结雷达工作参数。 */
struct ArDetectionCellConfig {
  double carrier_frequency_hz{0.0};
  double matched_filter_bandwidth_hz{0.0};
  double pulse_width_s{0.0};
  double radiated_peak_power_w{0.0};
  double one_way_antenna_gain_dbi{0.0};
  double receiver_loss_db{0.0};
  double receiver_noise_figure_db{0.0};
  double reference_temperature_k{290.0};
};

/** @brief 一个目标在本周期 detection cell 中的物理事实。 */
struct ArDetectionCellTarget {
  double range_m{0.0};
  double closing_radial_velocity_mps{0.0};
  double rcs_m2{0.0};
  double two_way_additional_propagation_loss_db{0.0};
  std::uint32_t effective_pulse_count{1U};
};

/** @brief 单目标 detection cell 的统计级求解结果。 */
struct ArDetectionCellResult {
  double echo_delay_s{0.0};
  double two_way_doppler_shift_hz{0.0};
  double echo_power_w{0.0};
  double pulse_compression_gain{1.0};
  double thermal_noise_power_w{0.0};
  double interference_power_w{0.0};
  double clutter_power_w{0.0};
  double processed_single_pulse_sinr_linear{0.0};
  double processed_single_pulse_sinr_db{0.0};
  std::uint32_t effective_pulse_count{1U};
};

/**
 * @brief 求解一个目标 detection cell；外部 incident link 以 W 聚合为压制干扰。
 * @param[in] own_emission_identity 当前 AR 发射身份；该链路不会计入干扰。
 * @param[in] clutter_power_w 当前 cell 的杂波功率（W）。
 * @return 成功返回 true；非法输入原子拒绝且不修改 @p result。
 */
bool TryResolveArDetectionCell(
    const ArDetectionCellConfig& config, const ArDetectionCellTarget& target,
    const oneq::electromagnetics::RfEmissionIdentity& own_emission_identity,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& incident_links,
    double clutter_power_w, ArDetectionCellResult* result);

}  // namespace detection
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_DETECTION_AR_DETECTION_CELL_RESOLVER_H_
