/**
 * @file ISignalPipeline.h
 * @brief 信号处理流水线内部端口（内部实现细节，不对外暴露）。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_PIPELINE_I_SIGNAL_PIPELINE_H_
#define AIRBORNE_RADAR_SIGNAL_PIPELINE_I_SIGNAL_PIPELINE_H_

#include <cstdint>
#include <memory>

#include "1q/airborne_radar/config/ArOrientationConfig.h"
#include "1q/airborne_radar/config/ArSessionConfig.h"
#include "1q/airborne_radar/session/ArControlProfile.h"
#include "1q/airborne_radar/session/ArInterferenceObservation.h"
#include "1q/airborne_radar/session/ArOutputTypes.h"
#include "1q/airborne_radar/session/ArSceneTypes.h"
#include "airborne_radar/environment/IEnvironmentService.h"

namespace airborne_radar {
namespace environment {
class IEnvironmentService;
}
namespace signal {

using session::AssociationQualityMetrics;
using session::SignalCycleAbortReason;
using session::SignalCycleResult;

struct SignalPipelineRuntimeState {
  const void* owner_identity{nullptr}; /**< 生成该快照的 pipeline 实例地址 */
  std::uint32_t schema_version{0U};    /**< 运行态快照 schema 版本 */
  std::shared_ptr<void> opaque{};      /**< 由具体 pipeline 实现解释的运行态快照 */
};

/**
 * @brief ISignalPipeline 定义单周期内的探测与跟踪处理内部端口。
 */
class ISignalPipeline {
 public:
  virtual ~ISignalPipeline() = default;

  /**
   * @brief 执行一次信号处理循环。
   * @param[in] scene_targets 当前周期场景目标输入列表。
   * @param[in] environment 环境服务只读接口。调用前必须已对该环境服务执行有效的
   *                        `BeginCycle(...)`，并确保其冻结快照携带正的 `cycle_dt_sec`。
   * @return 当前周期信号流水线输出结果。
   */
  virtual SignalCycleResult RunCycle(const session::ArSceneTargetList& scene_targets,
                                     const environment::IEnvironmentService& environment) = 0;

  /**
   * @brief 更新当前搭载平台姿态。
   * @param[in] platform_attitude_deg 平台姿态角（单位：度）。
   */
  virtual void UpdatePlatformAttitude(const config::PlatformAttitudeDeg& platform_attitude_deg) = 0;

  /**
   * @brief 更新当前雷达平台 WGS84 绝对海拔。
   * @param[in] platform_altitude_m 平台绝对海拔（单位：m）。
   */
  virtual void UpdatePlatformAltitudeM(float platform_altitude_m) { (void)platform_altitude_m; }

  /**
   * @brief 获取当前搭载平台姿态。
   * @return 平台姿态角（单位：度）。
   */
  virtual config::PlatformAttitudeDeg GetPlatformAttitude() const = 0;

  /**
   * @brief 获取当前缓存的雷达平台 WGS84 绝对海拔。
   * @return 平台绝对海拔（单位：m）。
   */
  virtual float GetPlatformAltitudeM() const { return 0.0f; }

  /**
   * @brief 设置下一周期生效的控制真值。
   * @param[in] control_profile 控制真值。
   */
  virtual void SetControlProfile(const session::ArControlProfile& control_profile) = 0;

  /**
   * @brief 获取当前缓存的控制真值。
   * @return 当前缓存的控制真值。
   */
  virtual session::ArControlProfile GetControlProfile() const = 0;

  /**
   * @brief 更新流水线运行配置。
   * @param[in] config 四域会话配置。
   * @return 配置已被接受并同步成功时返回 true；若当前实例拒绝该配置则返回 false。
   */
  virtual bool UpdateConfig(const config::ArSessionConfig& config) = 0;

  /**
   * @brief 获取上一周期关联质量指标。
   * @return 上一周期缓存的关联质量指标。
   */
  virtual AssociationQualityMetrics GetLastAssociationQualityMetrics() const = 0;

  /**
   * @brief 注入本周期待用的接收机干扰观测，供航迹起批阶段的假目标鉴别使用。
   *
   * 调用方在 @ref RunCycle 之前注入；pipeline 在量测构建阶段按方位将其中的假目标标注
   * 匹配到对应量测。默认空实现使不支持鉴别的 pipeline 实现免于改动。
   * @param[in] observations 干扰观测列表（按值持有，周期内消费后清空）。
   */
  virtual void SetPendingInterferenceObservations(
      session::ArInterferenceObservationList observations) {
    (void)observations;
  }

  /**
   * @brief 捕获当前 pipeline 运行态快照。
   * @return 可用于失败回滚的 pipeline 运行态快照。
   * @note 该内部端口用于控制器失败路径和回滚测试；快照必须完整覆盖会影响后续周期行为的
   *       运行态，失败周期会依赖该快照执行无损回滚。
   */
  virtual SignalPipelineRuntimeState CaptureRuntimeState() const = 0;

  /**
   * @brief 恢复此前捕获的 pipeline 运行态快照。
   * @param state 待恢复的 pipeline 运行态快照。
   * @note 恢复语义必须与 `CaptureRuntimeState()` 成对，确保失败周期不会留下内部游标、
   *       缓存或拓扑状态副作用。
   */
  virtual void RestoreRuntimeState(const SignalPipelineRuntimeState& state) = 0;
};

}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_PIPELINE_I_SIGNAL_PIPELINE_H_
