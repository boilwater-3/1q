/**
 * @file SarCycleInput.h
 * @brief 定义 SAR 会话单周期输入载荷。
 */

#ifndef ONEQ_SAR_SESSION_SAR_CYCLE_INPUT_H_
#define ONEQ_SAR_SESSION_SAR_CYCLE_INPUT_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"

namespace sar {
namespace session {

/**
 * @brief SAR 平台状态输入。
 * @note 内部生成 raw echo 时，位置、时间、NED 速度和姿态是本周期轨迹起点的权威输入；
 *       三轴速度全零明确表示静止。L3 waypoint 模式的位置轨迹仍由显式 waypoint 配置拥有
 *       authority。
 */
struct ONEQ_API SarPlatformState {
  double time_s{0.0};
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{0.0};
  double velocity_north_mps{0.0};
  double velocity_east_mps{0.0};
  double velocity_down_mps{0.0};
  double roll_deg{0.0};
  double pitch_deg{0.0};
  double yaw_deg{0.0};
};

/**
 * @brief SAR 点目标输入。
 */
struct ONEQ_API SarPointTarget {
  std::uint64_t target_id{0U};
  std::string target_name{};
  double latitude_deg{0.0};
  double longitude_deg{0.0};
  double altitude_m{0.0};
  double radar_cross_section_dbsm{0.0};
};

using SarPointTargetList = std::vector<SarPointTarget>;

/**
 * @brief 外部提供的完整孔径行主序复数 IQ 帧。
 * @note 当前支持 L1 RDA、双轨迹 L2+RDA 和实际轨迹 BP；replay 后续单独审批。
 *
 * 坐标系约定：`PulseState` 中的位置与速度使用 **本地直角坐标**（local Cartesian），
 * 与 `SarPlatformState` / `SarPointTarget` 的大地坐标（LLA + NED）不同。这是因为外部
 * IQ 数据通常已由上游系统在统一场景坐标系中处理完毕，直接以本地坐标提供可避免重复转换。
 *
 * 本地坐标系定义（与内部 `geometry::LocalPoint` 一致，采用 ENU 轴序）：
 * - 原点：`SarMissionConfig::scene_center_*` 对应的场景中心点。
 * - x 轴：东向（East），水平向东。
 * - y 轴：北向（North），水平向北。
 * - z 轴：上向（Up），垂直向上为正，相对 `scene_center_altitude_m`。
 *
 * 因此，调用方在填充 `pulse_states` 时必须使用 **相对于 `scene_center_*` 的 ENU 本地坐标**，
 * 而非绝对大地坐标。`SarSession` 内部会将这些值直接映射到聚焦算法所需的本地几何，
 * 不再做 LLA 转换。`SarExternalInputAdapter` 提供从 ECEF/LLA 到本坐标系的转换辅助。
 */
struct ONEQ_API SarRawIqFrame {
  /**
   * @brief 单脉冲平台状态（本地直角坐标，scene-center-relative ENU）。
   */
  struct PulseState {
    std::uint64_t pulse_id{0U};
    double time_s{0.0};
    double position_x_m{0.0};       /**< 东向位置（m），相对 scene_center */
    double position_y_m{0.0};       /**< 北向位置（m），相对 scene_center */
    double position_z_m{0.0};       /**< 上向位置（m），相对 scene_center_altitude_m */
    double velocity_x_mps{0.0};     /**< 东向速度（m/s） */
    double velocity_y_mps{0.0};     /**< 北向速度（m/s） */
    double velocity_z_mps{0.0};     /**< 上向速度（m/s） */
  };

  std::uint32_t pulse_count{0U};
  std::uint32_t samples_per_pulse{0U};
  std::vector<double> i_values{};
  std::vector<double> q_values{};
  // Actual platform trajectory associated with the IQ rows.
  std::vector<PulseState> pulse_states{};
  // Nominal trajectory required when external IQ requests L2 motion compensation.
  std::vector<PulseState> ideal_pulse_states{};
};

/**
 * @brief SAR 单周期输入。
 */
struct ONEQ_API SarCycleInput {
  std::uint32_t cycle_index{0U};
  float dt_sec{1.0f};
  SarPlatformState platform{};
  SarPointTargetList point_targets{};
  SarRawIqFrame raw_iq{};
};

}  // namespace session
}  // namespace sar

#endif  // ONEQ_SAR_SESSION_SAR_CYCLE_INPUT_H_
