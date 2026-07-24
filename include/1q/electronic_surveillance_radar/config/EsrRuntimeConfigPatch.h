/**
 * @file EsrRuntimeConfigPatch.h
 * @brief 定义 ESR 会话运行期配置补丁结构。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_RUNTIME_CONFIG_PATCH_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_RUNTIME_CONFIG_PATCH_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/config/EsrEnvironmentConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrMissionConfig.h"
#include "1q/electronic_surveillance_radar/config/EsrPolicyConfig.h"

namespace electronic_surveillance_radar {
namespace config {

/**
 * @brief ExplicitScanBounds 封装显式扫描边界相关的字段。
 *
 * 将 has_use_explicit_scan_bounds / use_explicit_scan_bounds 以及
 * 4 个 scan_*_deg 字段聚合成一个子结构，
 * 消除 has_use_explicit_scan_bounds 必须与其余字段同时设置的隐含耦合。
 */
struct ExplicitScanBounds {
  bool enabled{false};            /**< 是否使用显式扫描起止角 */
  float scan_start_az_deg{0.0f}; /**< 扫描起始方位角（单位：deg） */
  float scan_end_az_deg{0.0f};   /**< 扫描结束方位角（单位：deg） */
  float scan_start_el_deg{0.0f}; /**< 扫描起始俯仰角（单位：deg） */
  float scan_end_el_deg{0.0f};   /**< 扫描结束俯仰角（单位：deg） */
};

/**
 * @brief EsrEnvironmentRuntimeConfigPatch 描述运行期可变环境补丁。
 *
 * @note 环境预设（EsrEnvironmentPreset）是会话初始化语义，运行期不支持热更新。
 */
struct ONEQ_API EsrEnvironmentRuntimeConfigPatch {
  bool has_atmospheric_physics{false};
  EsrAtmosphericPhysicsConfig atmospheric_physics{};
};

/**
 * @brief EsrRuntimeConfigPatch 描述运行期可变参数补丁。
 *
 * 支持两类运行期更新：
 * 1) 整域覆盖：mission、policy、environment；
 * 2) 叶子覆盖：传感器开关、工作模式、扫描率、扫描中心、显式扫描边界等。
 * 其中 environment 仅允许模型叶子字段（如 atmospheric_physics），
 * 不支持 runtime preset 热更新。
 * 当整域与叶子同时出现时，先应用整域再应用叶子，叶子具有最终优先级。
 */
struct ONEQ_API EsrRuntimeConfigPatch {
  bool has_mission{false};            /**< [补丁标志] 是否整块覆盖任务域 */
  EsrMissionConfig mission{}; /**< [可外部调整] 任务域整块覆盖 */

  bool has_policy{false};           /**< [补丁标志] 是否整块覆盖策略域 */
  EsrPolicyConfig policy{}; /**< [可外部调整] 策略域整块覆盖 */

  bool has_environment{false}; /**< [补丁标志] 是否更新环境运行期配置 */
  EsrEnvironmentRuntimeConfigPatch environment{};

  bool has_sensor_enabled{false}; /**< [补丁标志] 是否显式设置传感器开关状态 */
  bool sensor_enabled{true};      /**< [可外部调整] 传感器开关状态 */

  bool has_work_mode{false};                                /**< [补丁标志] 是否显式设置工作模式 */
  EsrWorkMode work_mode{EsrWorkMode::kEsm}; /**< [可外部调整] 工作模式值 */

  bool has_scan_rate_hz{false}; /**< [补丁标志] 是否显式设置完整扫描图循环频率。 */
  float scan_rate_hz{1.0f};     /**< [可外部调整] 完整二维扫描图循环频率（单位：Hz）。 */

  bool has_scan_start_position{false}; /**< [补丁标志] 是否显式设置扫描起始位置 */
  EsrScanStartPosition scan_start_position{EsrScanStartPosition::kLeftTop};

  bool has_scan_sequence{false}; /**< [补丁标志] 是否显式设置扫描顺序 */
  EsrScanSequence scan_sequence{EsrScanSequence::kAzimuthFirst};

  bool has_scan_center_az_deg{false}; /**< [补丁标志] 是否显式设置扫描中心方位角 */
  float scan_center_az_deg{0.0f};     /**< [可外部调整] 扫描中心方位角（单位：deg） */

  bool has_scan_center_el_deg{false}; /**< [补丁标志] 是否显式设置扫描中心俯仰角 */
  float scan_center_el_deg{0.0f};     /**< [可外部调整] 扫描中心俯仰角（单位：deg） */

  bool has_explicit_scan_bounds{false};      /**< [补丁标志] 是否显式设置扫描边界 */
  ExplicitScanBounds explicit_scan_bounds{}; /**< [可外部调整] 显式扫描边界配置 */
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_RUNTIME_CONFIG_PATCH_H_
