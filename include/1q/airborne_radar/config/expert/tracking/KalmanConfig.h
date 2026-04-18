/**
 * @file KalmanConfig.h
 * @brief 定义 expert Kalman 配置。
 */

#ifndef AIRBORNE_RADAR_CONFIG_EXPERT_TRACKING_KALMAN_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_EXPERT_TRACKING_KALMAN_CONFIG_H_

namespace airborne_radar {
namespace config {
namespace expert {
namespace tracking {

/**
 * @brief Kalman 更新后端类型。
 */
enum class KalmanUpdateBackend {
  kStandardKfJoseph = 0, /**< 标准 Joseph 形式 KF。 */
  kUdKf = 1, /**< UD 分解 KF。 */
  kSrif = 2 /**< SRIF 后端。 */
};

/**
 * @brief expert Kalman 滤波参数。
 */
struct KalmanConfig {
  bool enable_kalman_filter{true}; /**< 是否启用 Kalman 滤波。 */
  float kalman_measurement_noise_std{10.0f}; /**< 量测噪声标准差。 */
  KalmanUpdateBackend kalman_update_backend{KalmanUpdateBackend::kStandardKfJoseph}; /**< 更新后端。 */
};

}  // namespace tracking
}  // namespace expert
}  // namespace config
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_EXPERT_TRACKING_KALMAN_CONFIG_H_
