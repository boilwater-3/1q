/**
 * @file pose_types.h
 * @brief 定义跨雷达模块复用的位姿轻量原语。
 */

#ifndef ONEQ_FOUNDATION_POSE_TYPES_H_
#define ONEQ_FOUNDATION_POSE_TYPES_H_

#include "1q/api.hpp"

namespace oneq {
namespace foundation {

/**
 * @brief Vector3f 表示三维向量（double 精度）。
 */
struct ONEQ_API Vector3f {
  double x{0.0}; /**< x 分量 */
  double y{0.0}; /**< y 分量 */
  double z{0.0}; /**< z 分量 */
};

/**
 * @brief EulerAnglesDeg 表示欧拉角姿态（单位：度，double 精度）。
 */
struct ONEQ_API EulerAnglesDeg {
  double yaw_deg{0.0};   /**< 偏航角（单位：deg） */
  double pitch_deg{0.0}; /**< 俯仰角（单位：deg） */
  double roll_deg{0.0};  /**< 横滚角（单位：deg） */

  EulerAnglesDeg() = default;
  EulerAnglesDeg(double yaw_deg_in, double pitch_deg_in, double roll_deg_in)
      : yaw_deg(yaw_deg_in), pitch_deg(pitch_deg_in), roll_deg(roll_deg_in) {}
};

/**
 * @brief PoseState 表示位置、速度与姿态的组合状态。
 */
struct ONEQ_API PoseState {
  Vector3f position_m{};         /**< 位置（单位：m） */
  Vector3f velocity_mps{};       /**< 速度（单位：m/s） */
  EulerAnglesDeg attitude_deg{}; /**< 姿态角（单位：deg） */
};

}  // namespace foundation
}  // namespace oneq

#endif  // ONEQ_FOUNDATION_POSE_TYPES_H_
