/**
 * @file IRadarContext.h
 * @brief 定义雷达系统对外暴露的上下文抽象接口。
 * 它解耦了中介者模式与环境状态之间的依赖。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTEXT_I_RADAR_CONTEXT_H_
#define AIRBORNE_RADAR_CORE_CONTEXT_I_RADAR_CONTEXT_H_

#include <cstdint>
#include <memory>
#include <vector>

#include "1q/airborne_radar/extension/control/RadarCommand.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/model/TargetFeature.h"
#include "1q/airborne_radar/model/RadarOrientationConfig.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/api.hpp"
#include "1q/foundation/pose_types.h"

namespace airborne_radar {
namespace extension {

struct RadarContextRuntimeState {
  const void* owner_identity{nullptr}; /**< 生成该快照的上下文实例地址 */
  std::uint32_t schema_version{0U};    /**< 快照 schema 版本 */
  std::shared_ptr<void> opaque{};      /**< 可选的实现私有快照负载；用于高效回滚 */
  model::TargetFeatureList target_features{};
  oneq::foundation::PoseState platform_pose{};
  float cycle_dt_sec{1.0f};
  std::vector<extension::control::RadarCommand> submitted_commands{};
  extension::control::RadarControlProfile latest_control_profile{};
  bool has_latest_control_profile{false};
};

/**
 * @brief IRadarContext 抽象了系统的当前战术态势操作。
 * 通过该接口，控制器不再依赖具体的处理层类，实现依赖倒置。
 */
class ONEQ_API IRadarContext {
 public:
  virtual ~IRadarContext() = default;

  /**
   * @brief 以单周期输入刷新上下文，并清空本周期输出缓存。
   * @param input 单周期输入载荷。
   */
  virtual void BeginCycle(const session::RadarCycleInput& input) = 0;

  /**
   * @brief 获取当前周期的目标特征列表。
   * @return 当前周期的目标特征列表只读引用。
   */
  virtual const model::TargetFeatureList& GetTargetFeatures() const = 0;

  /**
   * @brief 获取当前搭载平台姿态角。
   * @return 当前平台姿态角（单位：度）。
   */
  virtual model::PlatformAttitudeDeg GetPlatformAttitude() const = 0;

  /**
   * @brief 获取当前周期时间步长。
   * @return 当前周期时间步长（单位：秒）；<= 0 表示外部输入无效。
   */
  virtual float GetCycleDeltaTimeSec() const = 0;

  /**
   * @brief 提交（发射）一组战术动作命令给底座硬件或协调总线执行。
   * @param[in] cmd 单个被执行的控制器指令。
   */
  virtual void SubmitControlCommand(extension::control::RadarCommand cmd) = 0;

  /**
   * @brief 通知最新控制真值已生成。
   * @param[in] profile 下一周期控制真值。
   */
  virtual void UpdateRadarControlProfile(const extension::control::RadarControlProfile& profile) = 0;

  /**
   * @brief 获取当前周期已提交的控制指令。
   * @return 当前周期已提交的控制指令列表引用。
   */
  virtual const std::vector<extension::control::RadarCommand>& GetSubmittedCommands() const = 0;

  /**
   * @brief 判断是否已有最近一次控制真值。
   * @return 若已持有最近一次控制真值返回 true。
   */
  virtual bool HasLatestControlProfile() const = 0;

  /**
   * @brief 获取最近一次控制真值。
   * @return 最近一次控制真值引用。
   */
  virtual const extension::control::RadarControlProfile& GetLatestControlProfile() const = 0;

  /**
   * @brief 捕获当前上下文运行态快照。
   * @return 可用于失败回滚的上下文快照。
   * @note 默认回退字段为 `target_features/platform_pose/cycle_dt_sec/`
   *       `submitted_commands/latest_control_profile/has_latest_control_profile`。
   *       若实现需要降低快照开销，可在 `opaque` 中存放私有快照，并使用
   *       `owner_identity/schema_version` 防止跨实例或跨 schema 误用。
   */
  virtual RadarContextRuntimeState CaptureRuntimeState() const = 0;

  /**
   * @brief 恢复此前捕获的上下文运行态快照。
   * @param state 待恢复的运行态快照。
   * @note 若 `owner_identity/schema_version` 与当前实现匹配，优先恢复 `opaque` 中的
   *       私有快照；否则至少应正确回退公开字段，保证失败周期不遗留上下文副作用。
   */
  virtual void RestoreRuntimeState(const RadarContextRuntimeState& state) = 0;
};
}  // namespace extension
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTEXT_I_RADAR_CONTEXT_H_
