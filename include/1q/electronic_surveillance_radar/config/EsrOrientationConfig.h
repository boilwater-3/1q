/**
 * @file EsrOrientationConfig.h
 * @brief ESR 静态安装指向配置（条件五域第五域）。
 *
 * 会话顶层 `EsrSessionConfig::orientation` 仅承载天线相对平台参考轴的安装偏置。
 * 不进入 RuntimeConfigPatch；不引入 StabilizationMode / misalignment。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ORIENTATION_CONFIG_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ORIENTATION_CONFIG_H_

#include "1q/api.hpp"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief EsrOrientationConfig 描述 ESR 静态天线安装指向。
 * @note 初始化静态配置，不进入 RuntimeConfigPatch。
 * @note `ApplyScanPolicy` 从 mission 域扫描角中减去本域安装偏置，得到天线系扫描窗口。
 */
struct ONEQ_API EsrOrientationConfig {
  float antenna_mount_az_deg{0.0f}; /**< 天线中心方位相对角（单位：deg） */
  float antenna_mount_el_deg{0.0f}; /**< 天线中心俯仰相对角（单位：deg） */
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_ORIENTATION_CONFIG_H_
