/**
 * @file ControlCommandMapper.h
 * @brief 封装控制意图归并、状态镜像与命令提交职责。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTROLLER_CONTROL_COMMAND_MAPPER_H_
#define AIRBORNE_RADAR_CORE_CONTROLLER_CONTROL_COMMAND_MAPPER_H_

#include <vector>

#include "1q/airborne_radar/session/ArCommand.h"
#include "airborne_radar/decision/ControlReducerTypes.h"
#include "1q/airborne_radar/session/DecisionControlTypes.h"

namespace airborne_radar {
namespace decision {
class ControlReducer;
}  // namespace decision
namespace session {
class MutableArContext;
}  // namespace session

namespace extension {

/**
 * @brief 负责将战术 proposal 归并为控制真值并提交命令。
 *
 * 封装两个内聚职责：
 *   1. 调用 ControlReducer 归并 proposals -> profile；
 *   2. 将被采纳的 directives 转换为 ArCommand 并提交到 MutableArContext，
 *      同时更新控制真值。
 */
class ControlCommandMapper {
 public:
  /**
   * @brief 构造 mapper，所有依赖均为外部生命周期管理。
   * @param[in] control_reducer  控制归并器引用。
   * @param[in] radar_context    内部上下文，用于提交命令并更新 profile。
   * @note 传入的引用须在 mapper 生命周期内保持有效，mapper 不持有其所有权。
   */
  ControlCommandMapper(decision::ControlReducer& control_reducer,
                       session::MutableArContext& radar_context);

  /**
   * @brief 执行归并和命令提交。
   *
   * 调用后 *current_profile 更新为归并结果，radar_context 已收到所有命令和新 profile。
   *
   * @param[in,out] current_profile 当前控制真值指针，调用后被写入归并后的新值。
   * @param[in] proposals       本周期战术 proposal 列表。
   * @return 归并结果（包含新 profile、采纳与拒绝的 directives）。
   */
  extension::ControlReductionResult Apply(
      session::ArControlProfile* current_profile,
      const std::vector<session::TacticalProposal>& proposals);

  /** @brief 比较两个 profile，返回差异字段对应的 ControlDirective 列表。 */
  static std::vector<session::ControlDirective> DiffProfiles(
      const session::ArControlProfile& baseline,
      const session::ArControlProfile& target);

  /** @brief 将单个 ControlDirective 转换为 ArCommand。 */
  static session::ArCommand DirectiveToCommand(const session::ControlDirective& directive);

 private:
  decision::ControlReducer& control_reducer_;
  session::MutableArContext& radar_context_;
};

}  // namespace extension
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTROLLER_CONTROL_COMMAND_MAPPER_H_
