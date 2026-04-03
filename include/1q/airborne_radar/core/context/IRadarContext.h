/**
 * @file IRadarContext.h
 * @brief 定义雷达系统对外暴露的上下文抽象接口。
 * 它解耦了中介者模式与环境状态之间的依赖。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTEXT_I_RADAR_CONTEXT_H_
#define AIRBORNE_RADAR_CORE_CONTEXT_I_RADAR_CONTEXT_H_

#include <vector>

#include "1q/airborne_radar/common/control/RadarCommand.h"
#include "1q/airborne_radar/common/control/RadarControlProfile.h"
#include "1q/airborne_radar/common/model/TargetFeature.h"
#include "1q/airborne_radar/config/RadarOrientationConfig.h"
#include "1q/airborne_radar/core/context/RadarCycleInput.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace core {
namespace context {

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
  virtual void BeginCycle(const RadarCycleInput& input) = 0;

  /**
   * @brief 获取当前周期的目标特征列表。
   * @return 当前周期的目标特征列表只读引用。
   */
  virtual const common::model::TargetFeatureList& GetTargetFeatures() const = 0;

  /**
   * @brief 获取当前搭载平台姿态角。
   * @return 当前平台姿态角（单位：度）。
   */
  virtual common::config::PlatformAttitudeDeg GetPlatformAttitude() const = 0;

  /**
   * @brief 获取当前周期时间步长。
   * @return 当前周期时间步长（单位：秒）；<= 0 表示外部输入无效。
   */
  virtual float GetCycleDeltaTimeSec() const = 0;

  /**
   * @brief 提交（发射）一组战术动作命令给底座硬件或协调总线执行。
   * @param[in] cmd 单个被执行的控制器指令。
   */
  virtual void SubmitControlCommand(common::control::RadarCommand cmd) = 0;

  /**
   * @brief 通知最新控制真值已生成。
   * @param[in] profile 下一周期控制真值。
   */
  virtual void UpdateRadarControlProfile(const common::control::RadarControlProfile& profile) = 0;

  /**
   * @brief 获取当前周期已提交的控制指令。
   * @return 当前周期已提交的控制指令列表引用。
   */
  virtual const std::vector<common::control::RadarCommand>& GetSubmittedCommands() const = 0;

  /**
   * @brief 判断是否已有最近一次控制真值。
   * @return 若已持有最近一次控制真值返回 true。
   */
  virtual bool HasLatestControlProfile() const = 0;

  /**
   * @brief 获取最近一次控制真值。
   * @return 最近一次控制真值引用。
   */
  virtual const common::control::RadarControlProfile& GetLatestControlProfile() const = 0;
};
}  // namespace context
}  // namespace core
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTEXT_I_RADAR_CONTEXT_H_
