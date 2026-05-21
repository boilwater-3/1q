/**
 * @file RadarSession.h
 * @brief 定义面向外部接入的高层机载雷达会话门面。
 */

#ifndef AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_H_
#define AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_H_

#include <memory>
#include <vector>

#include "1q/airborne_radar/config/RadarSessionConfig.h"
#include "1q/airborne_radar/extension/control/RadarCommand.h"
#include "1q/airborne_radar/extension/control/RadarControlProfile.h"
#include "1q/airborne_radar/session/RadarCycleInput.h"
#include "1q/airborne_radar/session/RadarCycleResult.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace config {
struct RadarRuntimeConfigPatch;
}  // namespace config
}  // namespace airborne_radar

namespace airborne_radar {
namespace extension {
class IRadarContext;
class RadarController;
class ISignalPipeline;
}  // namespace extension
namespace environment {
class IEnvironmentService;
}
}  // namespace airborne_radar

namespace airborne_radar {
namespace session {
class RadarSessionFactory;
}
}  // namespace airborne_radar

namespace airborne_radar {
namespace session {

/**
 * @brief RadarSession 提供"一步一帧"的外部接入门面。
 */
class ONEQ_API RadarSession {
 public:
  /** @brief 使用默认配置构造会话。 */
  RadarSession();
  ~RadarSession();

  RadarSession(const RadarSession&) = delete;
  RadarSession& operator=(const RadarSession&) = delete;
  RadarSession(RadarSession&&) noexcept;
  RadarSession& operator=(RadarSession&&) noexcept;

  /**
   * @brief 执行一个不显式切场景的处理周期。
   * @param input 当前周期输入。
   * @return 当前周期生成的轨迹输出帧拷贝。
   * @note 输入容错：`dt_sec ≤ 0` 或其他非法输入时，函数不抛异常，
   *       会以尽力而为出发返回上一周期有效状态，输出帧中
   *       `tracks` 可能为空。
   */
  session::TrackOutputFrame Step(const RadarCycleInput& input);

  /**
   * @brief 执行一个不显式切场景的处理周期，并返回聚合结果。
   * @param input 当前周期输入。
   * @return 当前周期聚合结果。
   * @note 输入容错：行为与 `Step()` 一致；可通过结果中的
   *       `executed_this_cycle` / `reused_previous_track_output`
   *       区分“本周期实际执行”与“仅回退上一有效输出”；若下游主链路 abort，
   *       可进一步通过 `signal_cycle_abort_reason` 区分具体原因。
   */
  RadarCycleResult StepWithResult(const RadarCycleInput& input);

  /** @brief 获取当前周期已提交的控制指令。
   * @return 最近一次成功执行周期提交的控制指令列表引用。
   */
  const std::vector<extension::control::RadarCommand>& GetSubmittedCommands() const;

  /**
   * @brief 判断是否已经保存过最新控制真值。
   * @return 若已持有最近一次成功执行周期留下的控制真值则返回 true。
   */
  bool HasLatestControlProfile() const;

  /**
   * @brief 获取最近一次控制真值。
   * @return 最近一次成功执行周期留下的控制真值引用。
   */
  const extension::control::RadarControlProfile& GetLatestControlProfile() const;

  /**
   * @brief 获取最近一次关联质量观测指标。
   * @return 最近一次成功执行周期留下的关联质量观测指标。
   */
  extension::AssociationQualityMetrics GetLastAssociationQualityMetrics() const;

  /**
   * @brief 应用运行期可变配置补丁。
   * @param[in] patch 运行期可变配置补丁。
   * @note 该接口作为运行期可调参数的统一入口；未设置的字段保持现值不变。
   *       补丁会先进入 session 内部暂存区，并在下一次成功执行主链路的
   *       `Step()/StepWithResult()` 调用中最终提交；若本次调用在下游执行阶段失败，
   *       补丁仍保持 staged 状态。
   */
  void ApplyRuntimeConfig(const config::RadarRuntimeConfigPatch& patch);

 private:
  friend class RadarSessionFactory;

  struct Impl;
  explicit RadarSession(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_H_
