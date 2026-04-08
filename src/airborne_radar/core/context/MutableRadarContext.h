/**
 * @file MutableRadarContext.h
 * @brief 定义面向外部接入的可变雷达上下文默认实现。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTEXT_MUTABLE_RADAR_CONTEXT_H_
#define AIRBORNE_RADAR_CORE_CONTEXT_MUTABLE_RADAR_CONTEXT_H_

#include <vector>

#include "1q/airborne_radar/extension/IRadarContext.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"

namespace airborne_radar {
namespace session {

/**
 * @brief 提供一个可直接驱动控制器的默认雷达上下文实现。
 */
class MutableRadarContext final : public extension::IRadarContext {
 public:
  /**
   * @brief 默认构造函数。
   */
  MutableRadarContext() = default;
  ~MutableRadarContext() override = default;

  /**
   * @brief 以单周期输入刷新上下文，并清空本周期输出缓存。
   * @param input 单周期输入载荷。
   */
  void BeginCycle(const RadarCycleInput& input) override;

  /**
   * @brief 更新当前周期目标特征列表。
   * @param target_features 新的目标特征列表。
   */
  void SetTargetFeatures(model::TargetFeatureList target_features);

  /**
   * @brief 更新当前平台姿态角。
   * @param platform_attitude_deg 平台姿态角，单位为度。
   */
  void SetPlatformAttitude(const model::PlatformAttitudeDeg& platform_attitude_deg);

  /**
   * @brief 更新当前周期时间步长。
   * @param dt_sec 周期步长，单位为秒。
   */
  void SetCycleDeltaTimeSec(float dt_sec);

  /**
   * @brief 清空本周期输出缓存。
   * @note 当前仅清空已提交指令列表，不重置最近一次控制真值。
   */
  void ResetCycleOutputs();

  /**
   * @brief 获取本周期已提交的控制指令。
   * @return 当前周期命令缓存。
   */
  const std::vector<extension::control::RadarCommand>& GetSubmittedCommands() const override;

  /**
   * @brief 判断是否已经收到过控制真值更新。
   * @return 若至少收到过一次 `UpdateRadarControlProfile` 则返回 `true`。
   */
  bool HasLatestControlProfile() const override;

  /**
   * @brief 获取最近一次保存的控制真值。
   * @return 最近一次控制真值；若尚未更新则返回默认值。
   */
  const extension::control::RadarControlProfile& GetLatestControlProfile() const override;

  /**
   * @brief 获取当前周期的目标特征列表。
   * @return 当前周期的目标特征列表只读引用。
   */
  const model::TargetFeatureList& GetTargetFeatures() const override;

  /**
   * @brief 获取当前平台姿态角。
   * @return 当前平台姿态角。
   */
  model::PlatformAttitudeDeg GetPlatformAttitude() const override;

  /**
   * @brief 获取当前周期时间步长。
   * @return 当前周期步长，单位为秒。
   */
  float GetCycleDeltaTimeSec() const override;

  /**
   * @brief 记录控制器提交的单条控制指令。
   * @param cmd 控制指令。
   */
  void SubmitControlCommand(extension::control::RadarCommand cmd) override;

  /**
   * @brief 保存最近一次控制真值。
   * @param profile 下一周期控制真值。
   */
  void UpdateRadarControlProfile(const extension::control::RadarControlProfile& profile) override;

 private:
  model::TargetFeatureList target_features_{};
  model::PlatformAttitudeDeg platform_attitude_deg_{};
  float cycle_dt_sec_{1.0f};
  std::vector<extension::control::RadarCommand> submitted_commands_{};
  extension::control::RadarControlProfile latest_control_profile_{};
  bool has_latest_control_profile_{false};
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTEXT_MUTABLE_RADAR_CONTEXT_H_
