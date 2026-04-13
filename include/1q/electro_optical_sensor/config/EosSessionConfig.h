/**
 * @file EosSessionConfig.h
 * @brief 定义 EOS 会话初始化配置结构。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/config/EosWorkMode.h"
#include "1q/electro_optical_sensor/environment/EosEnvironmentConfig.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief EosSessionConfig 描述会话初始化参数。
 */
struct ONEQ_API EosSessionConfig {
  float wavelength_lower_um{3.0f};               /**< 工作波长下限（单位：um） */
  float wavelength_upper_um{5.0f};               /**< 工作波长上限（单位：um） */
  float optical_aperture_m{0.2f};                /**< 光学口径（单位：m） */
  float focal_length_m{0.8f};                    /**< 焦距（单位：m） */
  EosWorkMode work_mode{EosWorkMode::kFused};    /**< 工作模式 */
  float horizontal_fov_deg{6.0f};                /**< 水平视场角（单位：deg） */
  float vertical_fov_deg{4.0f};                  /**< 垂直视场角（单位：deg） */
  float scan_rate_deg_per_sec{20.0f};            /**< 扫描速率（单位：deg/s） */
  float frame_rate_hz{30.0f};                    /**< 帧频（单位：Hz） */
  float minimum_snr_db{6.0f};                    /**< 最低信噪比阈值（单位：dB） */
  float detection_sensitivity_w{1.0e-12f};       /**< 探测灵敏度（单位：W） */
  float scan_start_az_deg{-60.0f};               /**< 扫描起始方位角（单位：deg） */
  float scan_end_az_deg{60.0f};                  /**< 扫描结束方位角（单位：deg） */
  float scan_center_el_deg{0.0f};                /**< 扫描中心俯仰角（单位：deg） */
  float boresight_depression_deg{45.0f};         /**< 光轴下视角（单位：deg） */
  float min_detection_depression_deg{1.0f};      /**< 最小有效下视角（单位：deg） */
  float max_detection_depression_deg{89.0f};     /**< 最大有效下视角（单位：deg） */
  float visible_reference_irradiance_w_m2{800.0f}; /**< 可见光辐照度归一化参考值（单位：W/m^2） */
  bool enable_straylight_filter{false};          /**< 是否启用遮光罩杂散光抑制 */
  float hood_inner_half_angle_deg{12.0f};        /**< 遮光罩内半角（单位：deg） */
  float hood_outer_half_angle_deg{75.0f};        /**< 遮光罩外半角（单位：deg） */
  float hood_min_suppression_ratio{0.20f};       /**< 最低抑制比例，范围 [0, 1] */
  float hood_max_suppression_ratio{0.85f};       /**< 最高抑制比例，范围 [0, 1] */
  environment::EosEnvironmentDefaultConfig
      environment_default_config{}; /**< 默认环境配置 */
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_SESSION_CONFIG_H_
