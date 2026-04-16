/**
 * @file EsrScanPolicyConfig.h
 * @brief 定义 ESR 扫描策略配置。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SCAN_POLICY_CONFIG_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SCAN_POLICY_CONFIG_H_

#include "1q/api.hpp"
#include "1q/foundation/scan_schedule_types.h"

namespace electronic_surveillance_radar {
namespace config {

/** @brief ESR 扫描起始象限。 */
using EsrScanStartPosition = oneq::foundation::ScanStartPosition;

/** @brief ESR 二维扫描推进顺序。 */
using EsrScanSequence = oneq::foundation::ScanSequence;

/**
 * @brief EsrScanPolicyConfig 描述任务扫描语义输入。
 */
struct ONEQ_API EsrScanPolicyConfig {
  float scan_center_az_deg{0.0f};   /**< 扫描中心方位（单位：deg） */
  float scan_center_el_deg{0.0f};   /**< 扫描中心俯仰（单位：deg） */
  float scan_rate_hz{1.0f};         /**< 扫描数据率（单位：Hz） */
  EsrScanStartPosition scan_start_position{EsrScanStartPosition::kLeftTop}; /**< 扫描起始位置 */
  EsrScanSequence scan_sequence{EsrScanSequence::kAzimuthFirst};             /**< 扫描顺序 */
  bool use_explicit_scan_bounds{false}; /**< 是否使用显式扫描起止角 */
  float scan_start_az_deg{-60.0f};      /**< 扫描起始方位（单位：deg） */
  float scan_end_az_deg{60.0f};         /**< 扫描结束方位（单位：deg） */
  float scan_start_el_deg{-10.0f};      /**< 扫描起始俯仰（单位：deg） */
  float scan_end_el_deg{10.0f};         /**< 扫描结束俯仰（单位：deg） */
};

}  // namespace config
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_CONFIG_ESR_SCAN_POLICY_CONFIG_H_
