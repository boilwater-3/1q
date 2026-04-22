/**
 * @file EosMissionConfig.h
 * @brief 定义 EOS 任务域配置。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_MISSION_CONFIG_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_MISSION_CONFIG_H_

#include "1q/electro_optical_sensor/config/EosWorkMode.h"

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosMissionConfig 描述工作模式、扫描与指向任务参数。
 */
struct EosMissionConfig {
  config::EosWorkMode work_mode{config::EosWorkMode::kFused}; /**< 工作模式 */
  float horizontal_fov_deg{6.0f};                               /**< 水平视场角（单位：deg） */
  float vertical_fov_deg{4.0f};                                 /**< 垂直视场角（单位：deg） */
  float scan_rate_deg_per_sec{20.0f};                           /**< 扫描角速度（单位：deg/s） */
  float frame_rate_hz{30.0f};                                   /**< 帧率（单位：Hz） */
  float scan_start_az_deg{-60.0f};                              /**< 扫描起始方位角（单位：deg） */
  float scan_end_az_deg{60.0f};                                 /**< 扫描结束方位角（单位：deg） */
  float scan_center_el_deg{0.0f};                               /**< 扫描中心俯仰角（单位：deg） */
  float boresight_depression_deg{45.0f};                        /**< 视轴下俯角（单位：deg） */
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_MISSION_CONFIG_H_
