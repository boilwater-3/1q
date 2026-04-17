/**
 * @file EosPointingConfig.h
 * @brief 定义 EOS 指向策略配置。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_POINTING_CONFIG_H_
#define ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_POINTING_CONFIG_H_

namespace electro_optical_sensor {
namespace config {

/**
 * @brief EosPointingConfig 描述对外可配置的几何指向语义。
 */
struct EosPointingConfig {
  float scan_start_az_deg{-60.0f};       /**< 扫描起始方位角（单位：deg） */
  float scan_end_az_deg{60.0f};          /**< 扫描结束方位角（单位：deg） */
  float scan_center_el_deg{0.0f};        /**< 扫描中心俯仰角（单位：deg） */
  float boresight_depression_deg{45.0f}; /**< 视轴下俯角（单位：deg） */
};

}  // namespace config
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_CONFIG_EOS_POINTING_CONFIG_H_
