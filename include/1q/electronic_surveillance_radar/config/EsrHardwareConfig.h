/**
 * @file EsrHardwareConfig.h
 * @brief 定义 ESR 装备固有参数配置结构。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_HARDWARE_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_HARDWARE_CONFIG_H_

#include "1q/api.hpp"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrHardwareConfig 描述 ESR 装备固有参数。
 */
struct ONEQ_API EsrHardwareConfig {
  double receiver_band_lower_hz{0.23e9};  /**< 接收频段下限（单位：Hz） */
  double receiver_band_upper_hz{100.0e9}; /**< 接收频段上限（单位：Hz） */
  float receiver_sensitivity_w{1.0e-12f}; /**< 接收机灵敏度（单位：W） */
  float integrated_receive_loss_db{0.0f}; /**< 系统综合接收损耗（单位：dB） */
  float beam_az_width_deg{5.0f};          /**< 方位波束宽度（单位：deg） */
  float beam_el_width_deg{5.0f};          /**< 俯仰波束宽度（单位：deg） */
  float az_scan_range_deg{120.0f};        /**< 方位扫描范围（单位：deg） */
  float el_scan_range_deg{20.0f};         /**< 俯仰扫描范围（单位：deg） */
  float antenna_mount_az_deg{0.0f};       /**< 天线中心方位相对角（单位：deg） */
  float antenna_mount_el_deg{0.0f};       /**< 天线中心俯仰相对角（单位：deg） */
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_HARDWARE_CONFIG_H_
