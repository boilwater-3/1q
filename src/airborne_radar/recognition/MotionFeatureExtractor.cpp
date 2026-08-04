/**
 * @file MotionFeatureExtractor.cpp
 * @brief 运动特征提取器实现。
 */

#include "airborne_radar/recognition/MotionFeatureExtractor.h"

#include <algorithm>
#include <cmath>

namespace airborne_radar {
namespace recognition {

namespace {

float Clamp01(float value) { return std::max(0.0f, std::min(1.0f, value)); }

}  // namespace

MotionObservation MotionFeatureExtractor::Extract(const session::TrackStateSnapshot& snapshot,
                                                  float platform_altitude_m,
                                                  float estimation_uncertainty_trace) {
  MotionObservation observation;
  if (snapshot.status != session::TrackStatus::kConfirmed) {
    return observation;  // 识别仅以已确认航迹为对象；未确认仅积累不输出
  }

  observation.valid = true;
  observation.speed_mps = snapshot.speed;
  observation.acceleration_mps2 = snapshot.acceleration;
  observation.altitude_m = platform_altitude_m + snapshot.position_z;

  // 横向加速度分解：a_lat = |a - (a·v/|v|²)v|；低速或横向加速度低于阈值记为直线飞行。
  const float speed_sq = snapshot.velocity_x * snapshot.velocity_x +
                         snapshot.velocity_y * snapshot.velocity_y +
                         snapshot.velocity_z * snapshot.velocity_z;
  if (speed_sq > 1.0e-4f) {
    const float v_dot_a = snapshot.velocity_x * snapshot.acceleration_x +
                          snapshot.velocity_y * snapshot.acceleration_y +
                          snapshot.velocity_z * snapshot.acceleration_z;
    const float proj_x = v_dot_a / speed_sq * snapshot.velocity_x;
    const float proj_y = v_dot_a / speed_sq * snapshot.velocity_y;
    const float proj_z = v_dot_a / speed_sq * snapshot.velocity_z;
    const float lateral_x = snapshot.acceleration_x - proj_x;
    const float lateral_y = snapshot.acceleration_y - proj_y;
    const float lateral_z = snapshot.acceleration_z - proj_z;
    const float lateral_acceleration =
        std::sqrt(lateral_x * lateral_x + lateral_y * lateral_y + lateral_z * lateral_z);
    if (lateral_acceleration < 0.01f) {
      observation.is_straight = true;
    } else {
      observation.is_straight = false;
      observation.turn_radius_m = snapshot.speed * snapshot.speed / lateral_acceleration;
    }
  } else {
    observation.is_straight = true;
  }

  // 质量因子：估计不确定度越小质量越高（D2：P 的 position 分块迹作本源信号）。
  const float uncertainty_norm = std::max(0.0f, estimation_uncertainty_trace);
  observation.quality = Clamp01(10000.0f / (10000.0f + uncertainty_norm));
  return observation;
}

}  // namespace recognition
}  // namespace airborne_radar
