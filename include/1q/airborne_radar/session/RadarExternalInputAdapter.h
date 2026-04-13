/**
 * @file RadarExternalInputAdapter.h
 * @brief 机载雷达外部输入适配统一入口。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_RADAR_EXTERNAL_INPUT_ADAPTER_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_RADAR_EXTERNAL_INPUT_ADAPTER_H_

#include <cstdint>

#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/airborne_radar/model/TargetFeature.h"
#include "1q/api.hpp"
#include "1q/foundation/coordinate_transform.h"

namespace airborne_radar {
namespace session {

/**
 * @brief 描述外部坐标转换为雷达局部坐标所需的参考系信息。
 * @note `origin_lla` 定义局部 ENU 原点，`radar_attitude_deg` 定义雷达局部坐标相对 ENU 的姿态。
 */
struct ONEQ_API RadarLocalFrameReference {
  oneq::foundation::LlaCoordinateDegM origin_lla{};      /**< 雷达参考原点（WGS84 LLA） */
  oneq::foundation::EulerAnglesDeg radar_attitude_deg{}; /**< 雷达局部坐标相对 ENU 的姿态角 */
};

/**
 * @brief 目标速度输入参考系类型。
 */
enum class VelocityFrame {
  kRadarLocal = 0, /**< 雷达局部坐标速度（与 TargetFeature 的 position/velocity 同系） */
  kEcef = 1,       /**< 地固 ECEF 速度 */
  kEnu = 2,        /**< 局部 ENU 速度（x=east, y=north, z=up） */
  kNed = 3         /**< 局部 NED 速度（x=north, y=east, z=down） */
};

/**
 * @brief 外部目标运动学输入（统一入口）。
 */
struct ONEQ_API TargetExternalKinematicsInput {
  oneq::foundation::EcefCoordinateM radar_position_ecef_m{};       /**< 雷达位置（ECEF，m） */
  oneq::foundation::EcefCoordinateM target_position_ecef_m{};      /**< 目标位置（ECEF，m） */
  oneq::foundation::Vector3f target_velocity_mps{};                /**< 目标速度（单位：m/s） */
  VelocityFrame target_velocity_frame{VelocityFrame::kRadarLocal}; /**< 目标速度参考系 */

  bool has_radar_velocity_ecef_mps{false};              /**< 是否提供雷达自身 ECEF 速度 */
  oneq::foundation::Vector3f radar_velocity_ecef_mps{}; /**< 雷达自身 ECEF 速度（m/s） */

  model::EulerAnglesDeg platform_attitude_deg{};  /**< 平台姿态角（ENU->Body，deg） */
  model::EulerAnglesDeg radar_mount_angles_deg{}; /**< 雷达安装角（Body->Radar，deg） */
  float rcs{1.0f};                                /**< 目标 RCS（m^2） */
  int swerling_type{0};                           /**< 目标起伏模型 */
};

/**
 * @brief 严格复合平台姿态与雷达安装角，得到雷达局部姿态角。
 * @param[in] platform_attitude_deg 平台姿态角（ENU->Body，单位：deg）。
 * @param[in] radar_mount_angles_deg 雷达安装角（Body->Radar，单位：deg）。
 * @return 复合后的雷达姿态角（ENU->Radar，单位：deg）。
 * @note 该接口采用旋转矩阵复合，不使用欧拉角分量直接相加。
 */
ONEQ_API oneq::foundation::EulerAnglesDeg ComposeRadarAttitudeDeg(
    const model::EulerAnglesDeg& platform_attitude_deg,
    const model::EulerAnglesDeg& radar_mount_angles_deg);

/**
 * @brief 统一入口：由外部 ECEF 位置和多参考系速度构造 TargetFeature。
 * @param[in] external_target_id 外部目标标识符。
 * @param[in] input 外部运动学输入。
 * @param[out] target 输出目标特征，可为 nullptr。
 * @return 成功返回 true，输入非法或输出为空返回 false。
 *
 * @note 当 `has_radar_velocity_ecef_mps=true` 且速度输入为 ECEF/ENU/NED 时，
 *       会先扣除雷达自身速度，再转换到雷达局部坐标：
 *       `v_rel = v_target - v_radar`。
 */
ONEQ_API bool TryMakeTargetFromExternalKinematics(std::uint64_t external_target_id,
                                                  const TargetExternalKinematicsInput& input,
                                                  model::TargetFeature* target);

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_RADAR_EXTERNAL_INPUT_ADAPTER_H_
