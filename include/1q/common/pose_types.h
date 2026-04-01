/**
 * @file pose_types.h
 * @brief 定义跨雷达模块复用的位姿轻量原语。
 */

#ifndef ONEQ_COMMON_POSE_TYPES_H_
#define ONEQ_COMMON_POSE_TYPES_H_

#include "1q/api.hpp"

namespace oneq {
namespace common {

/**
 * @brief Vector3f 表示三维向量。
 */
struct ONEQ_API Vector3f {
  float x{0.0f}; /**< x 分量 */
  float y{0.0f}; /**< y 分量 */
  float z{0.0f}; /**< z 分量 */
};

/**
 * @brief EulerAnglesDeg 表示欧拉角姿态（单位：度）。
 */
struct ONEQ_API EulerAnglesDeg {
  float yaw_deg{0.0f};   /**< 偏航角（单位：deg） */
  float pitch_deg{0.0f}; /**< 俯仰角（单位：deg） */
  float roll_deg{0.0f};  /**< 横滚角（单位：deg） */
};

/**
 * @brief PoseState 表示位置、速度与姿态的组合状态。
 */
struct ONEQ_API PoseState {
  Vector3f position_m{};         /**< 位置（单位：m） */
  Vector3f velocity_mps{};       /**< 速度（单位：m/s） */
  EulerAnglesDeg attitude_deg{}; /**< 姿态角（单位：deg） */
};

}  // namespace common
}  // namespace oneq

#endif  // ONEQ_COMMON_POSE_TYPES_H_
