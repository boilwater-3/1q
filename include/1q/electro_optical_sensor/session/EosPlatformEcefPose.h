/**
 * @file EosPlatformEcefPose.h
 * @brief EOS 平台 ECEF 位姿（CycleInput 字段派生与输出反算共用）。
 *
 * 不是场景目标 ECEF→ENU 适配入口。场景目标请用公共
 * `oneq::coordinate::TryMakeEnuSceneState` 直填 `EosSceneTarget`。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_PLATFORM_ECEF_POSE_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_PLATFORM_ECEF_POSE_H_

#include "1q/api.hpp"
#include "1q/coordinate/types.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief 平台 ECEF 运动学与姿态（Body→ENU）。
 * @note 速度固定为 ECEF；EOS 无独立 mount 字段，视轴按与机体系对齐处理。
 */
struct ONEQ_API EosPlatformEcefPose {
  oneq::coordinate::EcefPositionM platform_position_ecef_m{}; /**< 平台位置（ECEF，m） */
  oneq::coordinate::EcefVelocityMps platform_velocity_mps{};  /**< 平台速度（ECEF，m/s） */
  oneq::coordinate::EulerAnglesDeg platform_attitude_deg{};   /**< 平台姿态角（Body→ENU，deg） */
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_PLATFORM_ECEF_POSE_H_
