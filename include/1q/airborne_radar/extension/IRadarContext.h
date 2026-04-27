/**
 * @file IRadarContext.h
 * @brief 定义雷达系统对外暴露的复合上下文抽象接口。
 *        继承自只读状态接口 IRadarContextReader、指令总线接口 IRadarCommandBus、
 *        控制真值接口 IRadarControlProfileStore，并增加生命周期管理方法。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTEXT_I_RADAR_CONTEXT_H_
#define AIRBORNE_RADAR_CORE_CONTEXT_I_RADAR_CONTEXT_H_

#include <cstdint>
#include <memory>
#include <vector>

#include "1q/airborne_radar/extension/IRadarCommandBus.h"
#include "1q/airborne_radar/extension/IRadarContextReader.h"
#include "1q/airborne_radar/extension/IRadarControlProfileStore.h"
#include "1q/airborne_radar/extension/control/RadarCommand.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/api.hpp"
#include "1q/foundation/pose_types.h"

namespace airborne_radar {
namespace extension {

struct ONEQ_API RadarContextRuntimeState {
  const void* owner_identity{nullptr}; /**< 生成该快照的上下文实例地址 */
  std::uint32_t schema_version{0U};    /**< 快照 schema 版本 */
  std::shared_ptr<void> opaque{};      /**< 可选的实现私有快照负载；用于高效回滚 */
  session::RadarSceneTargetList scene_targets{};
  oneq::foundation::PoseState platform_pose{};
  float cycle_dt_sec{1.0f};
  std::vector<extension::control::RadarCommand> submitted_commands{};
  extension::control::RadarControlProfile latest_control_profile{};
  bool has_latest_control_profile{false};
};

/**
 * @brief IRadarContext 组合了只读状态、指令总线、控制真值与生命周期管理。
 */
class ONEQ_API IRadarContext : public IRadarContextReader,
                               public IRadarCommandBus,
                               public IRadarControlProfileStore {
 public:
  ~IRadarContext() override = default;

  /**
   * @brief 以单周期输入刷新上下文，并清空本周期输出缓存。
   * @param input 单周期输入载荷。
   */
  virtual void BeginCycle(const session::RadarCycleInput& input) = 0;

  /**
   * @brief 捕获当前上下文运行态快照。
   * @return 可用于失败回滚的上下文快照。
   */
  virtual RadarContextRuntimeState CaptureRuntimeState() const = 0;

  /**
   * @brief 恢复此前捕获的上下文运行态快照。
   * @param state 待恢复的运行态快照。
   */
  virtual void RestoreRuntimeState(const RadarContextRuntimeState& state) = 0;
};

}  // namespace extension
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTEXT_I_RADAR_CONTEXT_H_
