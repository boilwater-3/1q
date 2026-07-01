/**
 * @file ControlReducerTypes.h
 * @brief 定义控制归并器内部使用的配置与结果类型。
 *
 * 本头已从 public API 收口为内部实现细节。ControlReducerConfig 的唯一对外
 * 触点 RadarController::UpdateControlReducerConfig() 已随本次收口移除；
 * reducer 退回全默认值构造，运行期不再可调参。
 */

#ifndef AIRBORNE_RADAR_DECISION_CONTROL_REDUCER_TYPES_H_
#define AIRBORNE_RADAR_DECISION_CONTROL_REDUCER_TYPES_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/session/ControlDirective.h"
#include "1q/airborne_radar/session/ArControlProfile.h"

namespace airborne_radar {
namespace extension {

/**
 * @brief ControlReducerConfig 描述 proposal -> profile 的固定映射与冲突裁决策略。
 */
struct ControlReducerConfig {
  float lpi_power_scale_on_reduction{0.5f}; /**< LPI 降功率意图映射到的默认功率比例 */
  float lpi_dwell_scale{0.75f};             /**< LPI 驻留调整意图映射到的默认驻留比例 */
  float eccm_burnthrough_gain{1.5f};        /**< ECCM 烧穿意图映射到的默认增益倍率 */
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
  session::RadarControlProfile profile; /**< 归并后的下一周期控制真值 */
  std::vector<session::ControlDirective> applied_directives;  /**< 被采纳的控制意图 */
  std::vector<session::ControlDirective> rejected_directives; /**< 被拒绝的控制意图 */
};

}  // namespace extension
}  // namespace airborne_radar

#endif  // ONEQ_AIRBORNE_RADAR_EXTENSION_CONTROL_REDUCER_TYPES_H_
