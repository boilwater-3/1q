/**
 * @file TrackingConfig.h
 * @brief 定义 expert 跟踪配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_EXPERT_TRACKING_TRACKING_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_EXPERT_TRACKING_TRACKING_CONFIG_H_

#include "1q/airborne_radar/config/expert/tracking/KalmanConfig.h"

namespace airborne_radar {
namespace config {
namespace expert {
namespace tracking {

/**
 * @brief expert 跟踪参数。
 */
struct TrackingConfig {
  bool enable_kalman_filter{true}; /**< 是否启用 Kalman 滤波。 */
  float kalman_measurement_noise_std{10.0f}; /**< 量测噪声标准差。 */
  KalmanUpdateBackend kalman_update_backend{KalmanUpdateBackend::kStandardKfJoseph}; /**< 更新后端。 */
  float speed_decay_ratio_on_loss{1.0f}; /**< 丢失周期速度衰减系数（默认无衰减）。 */
  float rcs_decay_ratio_on_loss{1.0f}; /**< 丢失周期 RCS 衰减系数（默认无衰减）。 */
};

}  // namespace tracking
}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_EXPERT_TRACKING_TRACKING_CONFIG_H_
