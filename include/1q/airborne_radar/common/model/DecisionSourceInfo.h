/**
 * @file DecisionSourceInfo.h
 * @brief 定义决策控制模块消费的来源信息结构体。
 */

#ifndef AIRBORNE_RADAR_COMMON_DECISION_SOURCE_INFO_H_
#define AIRBORNE_RADAR_COMMON_DECISION_SOURCE_INFO_H_

#include <vector>

namespace airborne_radar {
namespace common {
namespace model {

/**
 * @brief LpiSourceInfo 表示供 LPI 模块消费的来源信息。
 * TODO 当前仅保留一个布尔量，后续根据需求扩展为更细粒度威胁来源描述。
 */
struct LpiSourceInfo {
  bool has_recon_platform{false}; /**< 当前周期目标中是否存在电子侦察飞行器或侦察类设备 */
};

/**
 * @brief JammingTechnique 表示供决策层消费的干扰技术类型。
 */
enum class JammingTechnique {
  kUnknown = 0,      /**< 未知或未分类干扰 */
  kNoiseSuppression, /**< 压制式/噪声式干扰 */
  kDeception,        /**< 欺骗式干扰 */
  kRepeater          /**< 转发式/重复器式干扰 */
};

/**
 * @brief EccmJammerSourceInfo 表示单个干扰源的摘要事实。
 */
struct EccmJammerSourceInfo {
  JammingTechnique technique{JammingTechnique::kUnknown}; /**< 干扰技术类型 */
  float jammer_power_db{0.0f};                            /**< 干扰功率估计（单位：dB） */
  float jammer_to_signal_db{0.0f};                        /**< 干扰与信号比估计（单位：dB） */
  float frequency_overlap_ratio{0.0f}; /**< 干扰与当前工作频率的重叠度，范围 [0, 1] */
  float prf_lock_risk{0.0f};           /**< 干扰对当前 PRF 锁定的风险度，范围 [0, 1] */
  float azimuth_deg{0.0f};             /**< 干扰来向方位角（单位：deg） */
  float elevation_deg{0.0f};           /**< 干扰来向俯仰角（单位：deg） */
  float angular_span_deg{0.0f};        /**< 干扰角域宽度（单位：deg） */
  bool jammer_in_sidelobe{false};      /**< 干扰是否主要经由旁瓣进入 */
  float confidence{1.0f};              /**< 干扰事实置信度，范围 [0, 1] */
};

/** @brief 供 ECCM 消费的多源干扰摘要列表 */
using EccmJammerSourceInfoList = std::vector<EccmJammerSourceInfo>;

/**
 * @brief EccmSourceInfo 表示供 ECCM 模块消费的干扰来源信息。
 */
struct EccmSourceInfo {
  bool has_jamming_signal{false};            /**< 当前周期是否检测到干扰信号 */
  EccmJammerSourceInfoList jammer_sources{}; /**< 当前周期可见的多源干扰摘要 */

  EccmSourceInfo() = default; /**< 默认构造函数 */

  /**
   * @brief 带参数构造函数。
   * @param[in] has_jamming 当前周期是否检测到干扰信号。
   */
  explicit EccmSourceInfo(bool has_jamming) /**< 带参数构造函数 */
      : has_jamming_signal(has_jamming) {}
};

}  // namespace model
}  // namespace common
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_COMMON_DECISION_SOURCE_INFO_H_
