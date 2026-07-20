/**
 * @file SbirsRuntimeConfigImpact.h
 * @brief 描述 SBIRS runtime patch 对 pipeline 累积状态的最小迁移影响。
 */

#ifndef ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_RUNTIME_CONFIG_IMPACT_H_
#define ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_RUNTIME_CONFIG_IMPACT_H_

namespace sbirs_sensor {
namespace runtime {

/** @brief resolver 根据旧、新配置差异生成的内部状态迁移指令。 */
struct SbirsRuntimeConfigImpact {
  bool scan_sector_changed{false};
  bool reset_measurement_random_stream{false};
  bool reset_nis_gate_counts{false};
  bool reset_nfov_gate_failure_counts{false};
  bool restart_pointing_disturbance{false};
  bool release_estimated_tracks{false};
  bool nfov_channel_count_changed{false};
  bool clear_for_inactive{false};
  bool clear_for_wide_search{false};
  int previous_nfov_channel_count{1};
  int next_nfov_channel_count{1};
};

}  // namespace runtime
}  // namespace sbirs_sensor

#endif  // ONEQ_SRC_SBIRS_SENSOR_RUNTIME_SBIRS_RUNTIME_CONFIG_IMPACT_H_
