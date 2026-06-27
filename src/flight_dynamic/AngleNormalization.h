/**
 * @file AngleNormalization.h
 * @brief 角度归一化单一源：NormalizeRad（→[-π,π]）与 RadToDeg360（→[0,360)）。
 *
 * Autopilot 与 Maneuver（guidance）原先各自维护一份逐字相同的 NormalizeRad，
 * 是漂移源头。本单元将其唯一化为一份，逐字复刻既有 while-loop 实现（不改算法、
 * 不换 fmod），以保证行为零变化。RadToDeg360 原仅在 autopilot，一并收敛。
 *
 * 本单元为模块内部私有，不对外暴露。仅使用 C++11。
 */

#ifndef FLIGHT_DYNAMIC_ANGLE_NORMALIZATION_H_
#define FLIGHT_DYNAMIC_ANGLE_NORMALIZATION_H_

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace oneq {
namespace flight_dynamic {

/**
 * @brief 将弧度角归一化到 [-π, π]。
 *
 * 用 while 循环加减 2π（与既有 Autopilot/Maneuver 实现逐字一致，不改 fmod）。
 * 大幅超界输入（如 >>2π）会多次循环；调用方均为近界值。
 *
 * @param angle_rad 输入弧度。
 * @return 归一化到 [-π, π] 的弧度。
 */
inline double NormalizeRad(double angle_rad) {
  while (angle_rad > M_PI) angle_rad -= 2.0 * M_PI;
  while (angle_rad < -M_PI) angle_rad += 2.0 * M_PI;
  return angle_rad;
}

/**
 * @brief 将弧度角转为度并归一化到 [0, 360)。
 *
 * 先 rad→deg，再用 while 循环加减 360（与既有 Autopilot 实现逐字一致）。
 *
 * @param angle_rad 输入弧度。
 * @return 归一化到 [0, 360) 的度。
 */
inline double RadToDeg360(double angle_rad) {
  double deg = angle_rad * 180.0 / M_PI;
  while (deg < 0.0) deg += 360.0;
  while (deg >= 360.0) deg -= 360.0;
  return deg;
}

}  // namespace flight_dynamic
}  // namespace oneq

#endif  // FLIGHT_DYNAMIC_ANGLE_NORMALIZATION_H_
