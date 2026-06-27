/**
 * @file MutableRadarContext.h
 * @brief 定义面向外部接入的可变雷达上下文默认实现。
 */

#ifndef AIRBORNE_RADAR_CORE_CONTEXT_MUTABLE_RADAR_CONTEXT_H_
#define AIRBORNE_RADAR_CORE_CONTEXT_MUTABLE_RADAR_CONTEXT_H_

#include <memory>
#include <vector>

#include "1q/airborne_radar/session/RadarCommand.h"
#include "1q/airborne_radar/session/RadarControlProfile.h"
#include "1q/airborne_radar/config/RadarOrientationConfig.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/foundation/pose_types.h"

namespace airborne_radar {
namespace session {

struct RadarContextRuntimeState {
  const void* owner_identity{nullptr};
  std::uint32_t schema_version{0U};
  std::shared_ptr<void> opaque{};
  RadarSceneTargetList scene_targets{};
  oneq::foundation::PoseState platform_pose{};
  float platform_altitude_m{0.0f};
  float cycle_dt_sec{1.0f};
  std::uint32_t cycle_index{0U};
  std::vector<session::RadarCommand> submitted_commands{};
  session::RadarControlProfile latest_control_profile{};
  bool has_latest_control_profile{false};
};

/**
 * @brief 提供一个可直接驱动控制器的默认雷达上下文实现。
 */
class MutableRadarContext final {
 public:
  /**
   * @brief 默认构造函数。
   */
  MutableRadarContext() = default;
  explicit MutableRadarContext(RadarSceneTargetList scene_targets);
  ~MutableRadarContext() = default;

  /**
   * @brief 以单周期输入刷新上下文，并清空本周期输出缓存。
   * @param input 单周期输入载荷。
   */
  void BeginCycle(const RadarCycleInput& input);

  /**
   * @brief 更新当前周期场景目标列表。
   * @param scene_targets 新的场景目标列表。
   */
  void SetSceneTargets(RadarSceneTargetList scene_targets);

  /**
   * @brief 更新当前平台姿态角。
   * @param platform_attitude_deg 平台姿态角，单位为度。
   */
  void SetPlatformAttitude(const config::PlatformAttitudeDeg& platform_attitude_deg);

  /**
   * @brief 更新当前周期时间步长。
   * @param dt_sec 周期步长，单位为秒。
   */
  void SetCycleDeltaTimeSec(float dt_sec);

  /**
   * @brief 更新当前输入周期号。
   * @param cycle_index 当前周期号。
   */
  void SetCycleIndex(std::uint32_t cycle_index);

  /**
   * @brief 清空本周期输出缓存。
   * @note 当前仅清空已提交指令列表，不重置最近一次控制真值。
   */
  void ResetCycleOutputs();

  /**
   * @brief 获取本周期已提交的控制指令。
   * @return 当前周期命令缓存。
   */
  const std::vector<session::RadarCommand>& GetSubmittedCommands() const;

  const std::vector<session::RadarCommand>& SubmittedCommands() const;

  /**
   * @brief 判断是否已经收到过控制真值更新。
   * @return 若至少收到过一次 `UpdateRadarControlProfile` 则返回 `true`。
   */
  bool HasLatestControlProfile() const;

  /**
   * @brief 获取最近一次保存的控制真值。
   * @return 最近一次控制真值；若尚未更新则返回默认值。
   */
  const session::RadarControlProfile& GetLatestControlProfile() const;

  const session::RadarControlProfile& LatestControlProfile() const;

  RadarContextRuntimeState CaptureRuntimeState() const;

  void RestoreRuntimeState(const RadarContextRuntimeState& state);

  /**
   * @brief 获取当前周期场景目标列表。
   * @return 当前周期场景目标列表只读引用。
   */
  const RadarSceneTargetList& GetSceneTargets() const;

  /**
   * @brief 获取当前平台姿态角。
   * @return 当前平台姿态角。
   */
  config::PlatformAttitudeDeg GetPlatformAttitude() const;
  float GetPlatformAltitudeM() const;

  /**
   * @brief 获取当前周期时间步长。
   * @return 当前周期步长，单位为秒。
   */
  float GetCycleDeltaTimeSec() const;

  /**
   * @brief 获取当前输入周期号。
   * @return 当前周期号。
   */
  std::uint32_t GetCycleIndex() const;

  /**
   * @brief 记录控制器提交的单条控制指令。
   * @param cmd 控制指令。
   */
  void SubmitControlCommand(session::RadarCommand cmd);

  /**
   * @brief 保存最近一次控制真值。
   * @param profile 下一周期控制真值。
   */
  void UpdateRadarControlProfile(const session::RadarControlProfile& profile);

 private:
  struct RuntimeSnapshot;

  std::shared_ptr<RadarSceneTargetList> scene_targets_{new RadarSceneTargetList()};
  oneq::foundation::PoseState platform_pose_{};
  float platform_altitude_m_{0.0f};
  float cycle_dt_sec_{1.0f};
  std::uint32_t cycle_index_{0U};
  std::vector<session::RadarCommand> submitted_commands_{};
  session::RadarControlProfile latest_control_profile_{};
  bool has_latest_control_profile_{false};
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_CONTEXT_MUTABLE_RADAR_CONTEXT_H_
