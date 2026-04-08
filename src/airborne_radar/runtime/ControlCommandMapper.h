/**
 * @file ControlCommandMapper.h
 * @brief 封装控制意图归并、状态镜像与命令提交职责。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTROLLER_CONTROL_COMMAND_MAPPER_H_
#define AIRBORNE_RADAR_CORE_CONTROLLER_CONTROL_COMMAND_MAPPER_H_

#include <vector>

#include "1q/airborne_radar/extension/ControlReducerTypes.h"
#include "1q/airborne_radar/extension/IRadarContext.h"
#include "1q/airborne_radar/extension/ITacticalDecisionEngine.h"

namespace airborne_radar {
namespace decision {
namespace pipeline {
class ControlReducer;
}  // namespace pipeline
}  // namespace decision

namespace extension {

/**
 * @brief 负责将战术 proposal 归并为控制真值并提交命令。
 *
 * 封装三个内聚职责：
 *   1. 调用 ControlReducer 归并 proposals -> profile；
 *   2. 将 reducer 运行态镜像回 TacticalStateStore；
 *   3. 将被采纳的 directives 转换为 RadarCommand 并提交到 IRadarContext。
 */
class ControlCommandMapper {
 public:
  /**
   * @brief 构造 mapper，所有依赖均为外部生命周期管理。
   * @param control_reducer  控制归并器引用。
   * @param tactical_state_store 跨周期战术内存指针（可为 nullptr）。
   * @param radar_context    雷达上下文，用于提交命令和更新 profile。
   */
  ControlCommandMapper(decision::pipeline::ControlReducer& control_reducer,
                       extension::TacticalStateStore* tactical_state_store,
                       extension::IRadarContext& radar_context);

  /**
   * @brief 执行归并、状态镜像和命令提交。
   *
   * 调用后 *current_profile 更新为归并结果，IRadarContext 已收到新 profile 和所有命令。
   *
   * @param current_profile 当前控制真值指针，调用后被写入归并后的新值。
   * @param proposals       本周期战术 proposal 列表。
   * @return 归并结果（包含新 profile、采纳与拒绝的 directives）。
   */
  extension::ControlReductionResult Apply(
      extension::control::RadarControlProfile* current_profile,
      const std::vector<extension::TacticalProposal>& proposals);

 private:
  decision::pipeline::ControlReducer& control_reducer_;
  extension::TacticalStateStore* tactical_state_store_{nullptr};
  extension::IRadarContext& radar_context_;
};

}  // namespace extension
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTROLLER_CONTROL_COMMAND_MAPPER_H_
