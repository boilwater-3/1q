/**
 * @file EsrOrientationConfig.h
 * @brief 定义电子侦察雷达模块共享的姿态、位置与速度轻量类型。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_COMMON_ESR_ORIENTATION_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_COMMON_ESR_ORIENTATION_CONFIG_H_

#include "1q/api.hpp"

namespace electronic_surveillance_radar {
namespace common {

/**
 * @brief EsrVector3f 描述三维向量。
 */
struct ONEQ_API EsrVector3f {
  float x{0.0f}; /**< x 分量 */
  float y{0.0f}; /**< y 分量 */
  float z{0.0f}; /**< z 分量 */
};

/**
 * @brief EsrEulerAngleDeg 描述欧拉角姿态（单位：度）。
 */
struct ONEQ_API EsrEulerAngleDeg {
  float yaw_deg{0.0f}; /**< 偏航角（单位：deg） */
  float pitch_deg{0.0f}; /**< 俯仰角（单位：deg） */
  float roll_deg{0.0f}; /**< 横滚角（单位：deg） */
};

/**
 * @brief EsrPoseState 描述平台姿态与运动状态。
 */
struct ONEQ_API EsrPoseState {
  EsrVector3f position_m{}; /**< 平台位置（单位：m） */
  EsrVector3f velocity_mps{}; /**< 平台速度（单位：m/s） */
  EsrEulerAngleDeg attitude_deg{}; /**< 平台姿态角（单位：deg） */
};

}  // namespace common
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_COMMON_ESR_ORIENTATION_CONFIG_H_
