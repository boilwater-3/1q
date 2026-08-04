/**
 * @file ArSessionConfigValidation.h
 * @brief 机载雷达会话配置校验入口。
 *
 * 会话配置静态校验的主头文件。
 */

#ifndef ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_VALIDATION_H_
#define ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_VALIDATION_H_

#include <string>
#include <vector>

#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {

/**
 * @brief 会话配置校验问题分类码。
 *
 * 校验通过时返回空列表；存在问题时每个条目对应一项配置缺陷。
 */
enum class ConfigValidationCode {
  kNone = 0,                            /**< 无问题（默认占位）。 */
  kCommandedBeamwidthAzNotPositive,     /**< 指令态方位波束宽度启用时非有限或非正。 */
  kCommandedBeamwidthElNotPositive,     /**< 指令态俯仰波束宽度启用时非有限或非正。 */
  kMechanicalScanLimitsSwappedAz,       /**< 机械方位扫描下限大于上限。 */
  kMechanicalScanLimitsSwappedEl,       /**< 机械俯仰扫描下限大于上限。 */
  kElectronicScanLimitsSwappedAz,       /**< 电子方位扫描下限大于上限。 */
  kElectronicScanLimitsSwappedEl,       /**< 电子俯仰扫描下限大于上限。 */
  kAntennaAzGeometryInvalid,            /**< 方位名义波束宽度或物理孔径无法解析。 */
  kAntennaElGeometryInvalid,            /**< 俯仰名义波束宽度或物理孔径无法解析。 */
  kTransmitterFrequencyInvalid,         /**< 发射频率非有限或非正。 */
  kFrequencyPlanInvalid,                /**< 显式频率表为空、含非法值或不含初始载频。 */
  kTransmitterOperatingEnvelopeInvalid, /**< 发射功率、占空比或脉冲能量越界。 */
  kEquipmentIdentityInvalid,            /**< 发射与接收设备身份非法或冲突。 */
  kReceiverRfHardwareInvalid,           /**< 接收极化、隔离或线性边界非法。 */
  kRecognitionWeightsInvalid,           /**< 识别特征权重分量越界或权重和不为 1。 */
  kRecognitionDatabasePathMissing,      /**< 识别启用时数据库路径为空。 */
  kRecognitionThresholdInvalid,         /**< 识别判定门限（acceptance_score/minimum_margin）越界。 */
  kRecognitionAccumulationInvalid,      /**< 识别积累参数（min_confirmed_hits/min_observation_count）非法。 */
  kRecognitionTimeRangeInvalid          /**< 识别时间/作用范围参数（hold/max_range/dwell）非法。 */
};

/**
 * @brief 单条配置校验问题描述。
 */
struct ConfigValidationIssue {
  ConfigValidationCode code{ConfigValidationCode::kNone}; /**< 问题分类码。 */
  std::string field{};                                    /**< 触发问题的配置字段路径。 */
  std::string message{};                                  /**< 问题说明文本。 */
};

/** @brief 配置校验问题列表。 */
using ValidationIssueList = std::vector<ConfigValidationIssue>;

/**
 * @brief 对会话初始化配置执行静态结构校验。
 *
 * 校验范围：指令态波束宽度启用时须为有限正值；未启用指令态覆盖时，每个天线轴必须
 * 提供正的名义波束宽度，或提供可结合有效发射频率推导波束宽度的正物理孔径；
 * 发射频率须为有限正值；物理 RCS 评估频率须为 0 或有限正值；机械/电子扫描限位
 * 下限不大于上限。
 *
 * @param[in] config 待校验的会话初始化配置。
 * @return 校验问题列表；为空表示配置通过校验。
 * @note 该函数为 noexcept，仅做只读检查，不修改输入配置。
 */
ONEQ_API ValidationIssueList
ValidateArSessionConfig(const config::ArSessionConfig& config) noexcept;

}  // namespace config
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_CONFIG_AR_SESSION_CONFIG_VALIDATION_H_
