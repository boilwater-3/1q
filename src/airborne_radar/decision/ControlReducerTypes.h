/**
 * @file ControlReducerTypes.h
 * @brief 定义控制归并器内部使用的配置、结果与控制意图类型。
 *
 * 本头已从 public API 收口为内部实现细节。完整冲突裁决配置不对外暴露；
 * public `ArPolicyConfig::decision_control` 只承载四个 hold/cooldown 周期数，
 * 由 session 映射到本类型并可在成功周期提交边界更新。
 *
 * ControlDirective / ControlDirectiveType / ControlDirectiveSource / TacticalProposal
 * 原在公共头文件中定义，现移入此处，因为外部模块不再需要理解 proposal/directive 抽象
 * ——它们直接操作 ArControlProfile。
 */

#ifndef AIRBORNE_RADAR_DECISION_CONTROL_REDUCER_TYPES_H_
#define AIRBORNE_RADAR_DECISION_CONTROL_REDUCER_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/airborne_radar/session/ArControlProfile.h"

namespace airborne_radar {
namespace session {

/**
 * @brief ControlDirectiveSource 表示控制意图来源模块。
 */
enum class ControlDirectiveSource {
  UNKNOWN = 0,       /**< 未知来源 */
  THREAT_ASSESSMENT, /**< 威胁评估/分类来源 */
  EMISSION_CONTROL,  /**< 发射控制来源 */
  SURVIVABILITY      /**< 生存性/ECCM 来源 */
};

/**
 * @brief ControlDirectiveType 表示决策层输出的控制意图类型。
 */
enum class ControlDirectiveType {
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
struct ControlDirective {
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

/**
 * @brief TacticalProposal 表示一条带优先级和理由的控制意图提案。
 */
struct TacticalProposal {
  ControlDirective directive;
  int priority{0};
  std::string rationale;

  TacticalProposal() = default;
  TacticalProposal(const ControlDirective& proposal_directive, int proposal_priority,
                   const std::string& proposal_rationale)
      : directive(proposal_directive), priority(proposal_priority), rationale(proposal_rationale) {}
};

}  // namespace session

namespace extension {

/**
 * @brief ControlReducerConfig 描述 proposal -> profile 的固定映射与冲突裁决策略。
 */
struct ControlReducerConfig {
  float burnthrough_lpi_power_floor{
      0.85f}; /**< 当烧穿增益与 LPI 降功率并存时，对功率比例施加的保护下限 */
  std::uint32_t lpi_hold_cycles_after_request{0};  /**< LPI 域在收到 proposal 后额外保持的周期数 */
  std::uint32_t eccm_hold_cycles_after_request{0}; /**< ECCM 域在收到 proposal 后额外保持的周期数 */
  std::uint32_t lpi_cooldown_cycles_after_release{0};  /**< LPI 域释放后的冷却周期数 */
  std::uint32_t eccm_cooldown_cycles_after_release{0}; /**< ECCM 域释放后的冷却周期数 */
  bool prefer_survivability_in_power_conflict{true};   /**< 是否在烧穿/LPI 冲突时优先生存性 */
  bool prefer_survivability_in_beam_conflict{
      true}; /**< 是否在 LPI 波束形成与自适应波束形成冲突时优先生存性 */
};

/**
 * @brief ControlReductionResult 表示 reducer 的单周期输出。
 */
struct ControlReductionResult {
  session::ArControlProfile profile; /**< 归并后的下一周期控制真值 */
  std::vector<session::ControlDirective> applied_directives;  /**< 被采纳的控制意图 */
  std::vector<session::ControlDirective> rejected_directives; /**< 被拒绝的控制意图 */
};

}  // namespace extension
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_DECISION_CONTROL_REDUCER_TYPES_H_
