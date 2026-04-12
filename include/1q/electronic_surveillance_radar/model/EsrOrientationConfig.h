/**
 * @file EsrOrientationConfig.h
 * @brief 定义 ESR 对公共位姿原语的兼容类型别名。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_MODEL_ESR_ORIENTATION_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_MODEL_ESR_ORIENTATION_CONFIG_H_

#include "1q/common/pose_types.h"

namespace electronic_surveillance_radar {
namespace model {

/** @brief ESR 兼容别名：三维向量。 */
using EsrVector3f = oneq::common::Vector3f;

/** @brief ESR 兼容别名：欧拉角姿态（单位：deg）。 */
using EsrEulerAngleDeg = oneq::common::EulerAnglesDeg;

/** @brief ESR 兼容别名：位置、速度与姿态组合状态。 */
using EsrPoseState = oneq::common::PoseState;

}  // namespace model
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_MODEL_ESR_ORIENTATION_CONFIG_H_
