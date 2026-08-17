/**
 * @file RirCycleInput.h
 * @brief 远程识别雷达单周期输入类型。
 *
 * 周期输入（时间戳、平台状态、场景目标、RF 入射链路、环境快照）的主头文件。
 * 阶段 2-S 起为独立输入面：不再消费外部雷达航迹供给（RirTrackFeed 已退役）。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_CYCLE_INPUT_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_CYCLE_INPUT_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/electromagnetics/RfScene.h"
#include "1q/remote_identification_radar/config/RirEnvironmentConfig.h"
#include "1q/remote_identification_radar/session/RirSceneTypes.h"

namespace remote_identification_radar {
namespace session {

/**
 * @brief RirEnvironmentSnapshot 单周期环境快照（阶段 2-S S1）。
 *
 * `has_environment_data == false` 表示本周期未注入环境事实：传播损耗与杂波
 * 退化到阶段 1 旧口径（无额外传播损耗、无杂波），保证自持链路在最小输入
 * 下的行为可预测。
 */
struct ONEQ_API RirEnvironmentSnapshot {
  bool has_environment_data{false};   /**< 本周期是否携带环境事实。 */
  float weather_attenuation_db{0.0f}; /**< 天气附加双程传播损耗（dB），须有限且 ≥0。 */
  config::RirVegetationScatterPhysicsConfig vegetation_scatter_physics{}; /**< 植被场景事实。 */
};

/**
 * @brief RirCycleInput 描述单周期输入。
 *
 * @note 帧约定：`platform_altitude_m` 为平台绝对海拔（识别高度观测 =
 *       平台海拔 + 内部航迹 `position_z`，ENU 局部切平面上向分量）；
 *       `sim_time_sec` 为调用方提供的仿真时间。RF 输入为已求解的 incident
 *       links（common 单源）与 RIR 自身发射身份；RIR 不消费任何 AR 输出。
 */
struct ONEQ_API RirCycleInput {
  std::uint32_t input_cycle_index{0U}; /**< 本次调用输入周期号（≠0 合法）。 */
  std::uint64_t batch_id{0U};          /**< 输入批号。 */
  double dt_sec{0.0};                  /**< 周期步长（s），有限且为正。 */
  float sim_time_sec{0.0f};            /**< 当前仿真时间（s）。 */
  float platform_altitude_m{0.0f};     /**< 平台绝对海拔（m）。 */

  RirSceneTargetList scene_targets{};            /**< 场景目标（含识别特征真值与运动事实）。 */
  RirEnvironmentSnapshot environment_snapshot{}; /**< 环境快照（天气/植被）。 */

  oneq::electromagnetics::RfEmissionIdentity own_emission_identity{}; /**< RIR 自身发射身份。 */
  std::vector<oneq::electromagnetics::RfIncidentLinkResult> incident_links{}; /**< 单程入射链路。 */
};

}  // namespace session
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_SESSION_RIR_CYCLE_INPUT_H_
