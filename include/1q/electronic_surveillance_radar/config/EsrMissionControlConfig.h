/**
 * @file EsrMissionControlConfig.h
 * @brief 定义 ESR 任务运行态控制参数结构。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_MISSION_CONTROL_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_MISSION_CONTROL_CONFIG_H_

#include "1q/api.hpp"
#include "1q/common/scan_schedule_types.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief EsrWorkMode 描述 ESR 工作模式。
 */
enum class ONEQ_API EsrWorkMode {
  kEsm = 0, /**< 常规电子支援侦察模式 */
  kHgesm,   /**< 高增益电子支援侦察模式 */
  kRwr      /**< 告警接收机模式 */
};

/** @brief ESR 兼容别名：扫描起始象限。 */
using EsrScanStartPosition = oneq::common::ScanStartPosition;

/** @brief ESR 兼容别名：二维扫描推进顺序。 */
using EsrScanSequence = oneq::common::ScanSequence;

/**
 * @brief EsrMissionControlConfig 描述 ESR 任务运行态控制参数。
 */
struct ONEQ_API EsrMissionControlConfig {
  bool power_on{true};                      /**< 设备开关机状态 */
  EsrWorkMode work_mode{EsrWorkMode::kEsm}; /**< 当前工作模式 */
  float scan_center_az_deg{0.0f};           /**< 扫描中心方位（单位：deg） */
  float scan_center_el_deg{0.0f};           /**< 扫描中心俯仰（单位：deg） */
  float scan_rate_hz{1.0f};                 /**< 扫描数据率（单位：Hz） */
  EsrScanStartPosition scan_start_position{EsrScanStartPosition::kLeftTop}; /**< 扫描起始位置 */
  EsrScanSequence scan_sequence{EsrScanSequence::kAzimuthFirst};            /**< 扫描顺序 */
  bool use_explicit_scan_bounds{false}; /**< 是否使用显式扫描起止角 */
  float scan_start_az_deg{-60.0f};      /**< 扫描起始方位（单位：deg） */
  float scan_end_az_deg{60.0f};         /**< 扫描结束方位（单位：deg） */
  float scan_start_el_deg{-10.0f};      /**< 扫描起始俯仰（单位：deg） */
  float scan_end_el_deg{10.0f};         /**< 扫描结束俯仰（单位：deg） */
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_MISSION_CONTROL_CONFIG_H_
