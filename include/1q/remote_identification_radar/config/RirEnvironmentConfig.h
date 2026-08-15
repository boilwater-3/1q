/**
 * @file RirEnvironmentConfig.h
 * @brief 远程识别雷达环境域主配置类型。
 *
 * 当前识别链路不消费环境服务：效能级 SNR 由模块内雷达方程 + 热噪声底自算，
 * 无大气附加损耗输入（对齐 AR 审计"不引入未消费的死输入"教训）。
 * 本结构仅保持四域解剖完整性，扩展环境输入须先有真实消费路径。
 */

#ifndef ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_ENVIRONMENT_CONFIG_H_
#define ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_ENVIRONMENT_CONFIG_H_

#include "1q/api.hpp"

namespace remote_identification_radar {
namespace config {

/**
 * @brief RirEnvironmentConfig 远程识别雷达环境域配置（当前为空占位）。
 */
struct ONEQ_API RirEnvironmentConfig {};

}  // namespace config
}  // namespace remote_identification_radar

#endif  // ONEQ_REMOTE_IDENTIFICATION_RADAR_CONFIG_RIR_ENVIRONMENT_CONFIG_H_
