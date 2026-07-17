/**
 * @file EsrMissionConfig.h
 * @brief 定义 ESR 任务域配置、工作模式与扫描策略。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_MISSION_CONFIG_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_MISSION_CONFIG_H_

#include "1q/api.hpp"
#include "1q/foundation/scan_schedule_types.h"

namespace electronic_surveillance_radar {
namespace config {

/** @brief ESR 扫描起始象限。 */
using EsrScanStartPosition = oneq::foundation::ScanStartPosition;

/** @brief ESR 二维扫描推进顺序。 */
using EsrScanSequence = oneq::foundation::ScanSequence;

/**
 * @brief EsrWorkMode 描述 ESR 工作模式。
 */
enum class ONEQ_API EsrWorkMode {
  kEsm = 0, /**< 常规电子支援侦察模式 */
  kHgesm,   /**< 高增益电子支援侦察模式 */
  kRwr      /**< 告警接收机模式 */
};

/**
 * @brief EsrScanPolicyConfig 描述任务扫描语义输入。
 *
 * @note `use_explicit_scan_bounds == true` 时，四个显式起止角唯一生效，中心角字段被忽略；
 *       四个边界必须为有限值且每个轴满足 start < end。为 false 时，显式边界字段被忽略，
 *       中心角结合硬件扫描范围解析扫描区间。
 */
struct ONEQ_API EsrScanPolicyConfig {
  float scan_center_az_deg{0.0f};   /**< 扫描中心方位（单位：deg） */
  float scan_center_el_deg{0.0f};   /**< 扫描中心俯仰（单位：deg） */
  float scan_rate_hz{1.0f};         /**< 每秒完成的完整二维扫描图循环数（单位：Hz）。 */
  EsrScanStartPosition scan_start_position{EsrScanStartPosition::kLeftTop}; /**< 扫描起始位置 */
  EsrScanSequence scan_sequence{EsrScanSequence::kAzimuthFirst};             /**< 扫描顺序 */
  bool use_explicit_scan_bounds{false}; /**< 是否使用显式扫描起止角；启用后显式边界优先于中心角 */
  float scan_start_az_deg{-60.0f};      /**< 扫描起始方位（单位：deg） */
  float scan_end_az_deg{60.0f};         /**< 扫描结束方位（单位：deg） */
  float scan_start_el_deg{-10.0f};      /**< 扫描起始俯仰（单位：deg） */
  float scan_end_el_deg{10.0f};         /**< 扫描结束俯仰（单位：deg） */
};

/**
 * @brief EsrMissionConfig 描述 ESR 任务控制与扫描语义输入。
 */
struct ONEQ_API EsrMissionConfig {
  bool power_on{true};                      /**< 设备开关机状态 */
  EsrWorkMode work_mode{EsrWorkMode::kEsm}; /**< 当前工作模式 */
  EsrScanPolicyConfig scan{};               /**< 扫描策略语义输入 */
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_MISSION_CONFIG_H_
