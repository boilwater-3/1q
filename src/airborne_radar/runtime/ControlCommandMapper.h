/**
 * @file ControlCommandMapper.h
 * @brief 封装控制意图归并、状态镜像与命令提交职责。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTROLLER_CONTROL_COMMAND_MAPPER_H_
#define AIRBORNE_RADAR_CORE_CONTROLLER_CONTROL_COMMAND_MAPPER_H_

#include <vector>

#include "airborne_radar/decision/ControlReducerTypes.h"
#include "1q/airborne_radar/session/ITacticalDecisionEngine.h"

namespace airborne_radar {
namespace decision {
class ControlReducer;
}  // namespace decision
namespace session {
class MutableRadarContext;
}  // namespace session

namespace extension {

/**
 * @brief 负责将战术 proposal 归并为控制真值并提交命令。
 *
 * 封装两个内聚职责：
 *   1. 调用 ControlReducer 归并 proposals -> profile；
 *   2. 将被采纳的 directives 转换为 RadarCommand 并提交到 MutableRadarContext，
 *      同时更新控制真值。
 */
class ControlCommandMapper {
 public:
  /**
   * @brief 构造 mapper，所有依赖均为外部生命周期管理。
   * @param control_reducer  控制归并器引用。
   * @param radar_context    内部上下文，用于提交命令并更新 profile。
   */
  ControlCommandMapper(decision::ControlReducer& control_reducer,
                       session::MutableRadarContext& radar_context);

  /**
   * @brief 执行归并和命令提交。
   *
   * 调用后 *current_profile 更新为归并结果，radar_context 已收到所有命令和新 profile。
   *
   * @param current_profile 当前控制真值指针，调用后被写入归并后的新值。
   * @param proposals       本周期战术 proposal 列表。
   * @return 归并结果（包含新 profile、采纳与拒绝的 directives）。
   */
  extension::ControlReductionResult Apply(
      session::RadarControlProfile* current_profile,
      const std::vector<session::TacticalProposal>& proposals);

 private:
  decision::ControlReducer& control_reducer_;
  session::MutableRadarContext& radar_context_;
};

}  // namespace extension
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTROLLER_CONTROL_COMMAND_MAPPER_H_
