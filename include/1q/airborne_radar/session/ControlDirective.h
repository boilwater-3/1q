/**
 * @file ControlDirective.h
 * @brief 定义决策层输出的控制意图。
 */

#ifndef ONEQ_AIRBORNE_RADAR_SESSION_CONTROL_DIRECTIVE_H_
#define ONEQ_AIRBORNE_RADAR_SESSION_CONTROL_DIRECTIVE_H_

#include "1q/api.hpp"

namespace airborne_radar {
namespace session {

/**
 * @brief ControlDirectiveSource 表示控制意图来源模块。
 */
enum class ONEQ_API ControlDirectiveSource {
  UNKNOWN = 0,       /**< 未知来源 */
  THREAT_ASSESSMENT, /**< 威胁评估/分类来源 */
  EMISSION_CONTROL,  /**< 发射控制来源 */
  SURVIVABILITY      /**< 生存性/ECCM 来源 */
};

/**
 * @brief ControlDirectiveType 表示决策层输出的控制意图类型。
 */
enum class ONEQ_API ControlDirectiveType {
  NONE = 0,                            /**< 空意图 */
  REQUEST_LPI_POWER_REDUCTION,         /**< 请求降低发射功率 */
  REQUEST_LPI_BEAMFORMING,             /**< 请求启用 LPI 波束形成 */
  REQUEST_LPI_DWELL,                   /**< 请求调整 LPI 驻留参数 */
  REQUEST_ENABLE_SIDELOBE_CANCELLER,   /**< 请求启用旁瓣对消 */
  REQUEST_ENABLE_ADAPTIVE_BEAMFORMING, /**< 请求启用自适应波束形成 */
  REQUEST_AGILITY_FREQUENCY,           /**< 请求启用频率捷变 */
  REQUEST_ECCM_REJITTER,               /**< 请求启用重频抖动 */
  REQUEST_ECCM_BURNTHROUGH_GAIN,       /**< 请求提升烧穿增益 */
  REQUEST_ANTI_RGPO_LEADING_EDGE,      /**< 请求启用前沿跟踪对抗距离拖引 */
  REQUEST_ANTI_VGPO_ACCELERATION_BOUND, /**< 请求启用加速度限幅对抗速度拖引 */
  REQUEST_ANTI_FALSE_TARGET_DISCRIMINATION, /**< 请求启用假目标鉴别 */
  /**
   * @brief 哨兵值，表示 ControlDirectiveType 的有效取值数量。
   * @note 仅用于编译期/测试期穷尽性检查（如 switch 覆盖矩阵）。
   *       不得作为真实意图传递、不得序列化；其本身不参与 LPI/ECCM 域判定。
   */
  kCount
};

/**
 * @brief ControlDirective 表示一条可带附加信息的控制意图。
 */
struct ONEQ_API ControlDirective {
  ControlDirectiveType type{ControlDirectiveType::NONE};          /**< 控制意图类型 */
  ControlDirectiveSource source{ControlDirectiveSource::UNKNOWN}; /**< 控制意图来源 */
  bool has_requested_value{false}; /**< 是否携带该控制意图要求的显式标量 */
  float requested_value{0.0f};     /**< 功率/驻留比例或烧穿增益，语义由 type 决定 */

  ControlDirective() = default; /**< 默认构造 */

  /**
   * @brief 便捷构造控制意图。
   * @param[in] directive_type 控制意图类型。
   * @param[in] directive_source 控制意图来源。
   */
  ControlDirective(ControlDirectiveType directive_type, ControlDirectiveSource directive_source)
      : type(directive_type), source(directive_source) {}

  /** @brief 构造携带显式标量的控制意图。 */
  ControlDirective(ControlDirectiveType directive_type, ControlDirectiveSource directive_source,
                   float value)
      : type(directive_type),
        source(directive_source),
        has_requested_value(true),
        requested_value(value) {}
};

}  // namespace session
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_SESSION_CONTROL_DIRECTIVE_H_
