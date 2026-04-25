/**
 * @file ControlCommandMapper.h
 * @brief 封装控制意图归并、状态镜像与命令提交职责。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTROLLER_CONTROL_COMMAND_MAPPER_H_
#define AIRBORNE_RADAR_CORE_CONTROLLER_CONTROL_COMMAND_MAPPER_H_

#include <vector>

#include "1q/airborne_radar/extension/ControlReducerTypes.h"
#include "1q/airborne_radar/extension/IRadarCommandBus.h"
#include "1q/airborne_radar/extension/IRadarControlProfileStore.h"
#include "1q/airborne_radar/extension/ITacticalDecisionEngine.h"

namespace airborne_radar {
namespace decision {
class ControlReducer;
}  // namespace decision

namespace extension {

/**
 * @brief 负责将战术 proposal 归并为控制真值并提交命令。
 *
 * 封装两个内聚职责：
 *   1. 调用 ControlReducer 归并 proposals -> profile；
 *   2. 将被采纳的 directives 转换为 RadarCommand 并提交到 IRadarCommandBus，
 *      同时更新 IRadarControlProfileStore。
 */
class ControlCommandMapper {
 public:
  /**
   * @brief 构造 mapper，所有依赖均为外部生命周期管理。
   * @param control_reducer  控制归并器引用。
   * @param command_bus      指令总线，用于提交命令。
   * @param profile_store    控制真值存储，用于更新 profile。
   */
  ControlCommandMapper(decision::ControlReducer& control_reducer,
                       extension::IRadarCommandBus& command_bus,
                       extension::IRadarControlProfileStore& profile_store);

  /**
   * @brief 执行归并和命令提交。
   *
   * 调用后 *current_profile 更新为归并结果，command_bus 已收到所有命令，
   * profile_store 已收到新 profile。
   *
   * @param current_profile 当前控制真值指针，调用后被写入归并后的新值。
   * @param proposals       本周期战术 proposal 列表。
   * @return 归并结果（包含新 profile、采纳与拒绝的 directives）。
   */
  extension::ControlReductionResult Apply(
      extension::control::RadarControlProfile* current_profile,
      const std::vector<extension::TacticalProposal>& proposals);

 private:
  decision::ControlReducer& control_reducer_;
  extension::IRadarCommandBus& command_bus_;
  extension::IRadarControlProfileStore& profile_store_;
};

}  // namespace extension
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTROLLER_CONTROL_COMMAND_MAPPER_H_
