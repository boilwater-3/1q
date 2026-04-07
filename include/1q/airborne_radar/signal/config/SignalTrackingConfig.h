/**
 * @file SignalTrackingConfig.h
 * @brief 定义跟踪域关键配置（config 主入口）。
 */

#ifndef AIRBORNE_RADAR_CONFIG_SIGNAL_TRACKING_CONFIG_H_
#define AIRBORNE_RADAR_CONFIG_SIGNAL_TRACKING_CONFIG_H_

namespace airborne_radar {
namespace signal {
namespace config {

/**
 * @brief KalmanUpdateBackend 表示 Kalman 滤波更新后端类型。
 */
enum class KalmanUpdateBackend {
  kStandardKfJoseph = 0, /**< 标准 Kalman Joseph 更新 */
  kUdKf,                 /**< UD 分解稳定更新 */
  kSrif                  /**< SRIF 信息滤波更新 */
};

/**
 * @brief SignalTrackingConfig 描述信号层对外可调的跟踪域配置。
 */
struct SignalTrackingConfig {
  bool enable_kalman_filter{true};           /**< 是否启用卡尔曼滤波器 */
  float kalman_measurement_noise_std{10.0f}; /**< 卡尔曼测量噪声标准差 */
  KalmanUpdateBackend kalman_update_backend{
      KalmanUpdateBackend::kStandardKfJoseph}; /**< Kalman 更新后端类型 */
};

}  // namespace config
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CONFIG_SIGNAL_TRACKING_CONFIG_H_
