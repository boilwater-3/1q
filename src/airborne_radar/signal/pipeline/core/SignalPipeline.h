/**
 * @file SignalPipeline.h
 * @brief 定义信号处理流水线的默认内部实现。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_

#include <memory>
#include <vector>

#include "1q/airborne_radar/extension/ISignalPipeline.h"
#include "airborne_radar/signal/pipeline/config/SignalPipelineExecutionConfig.h"
#include "airborne_radar/signal/tracking/ITrackLifecycleManager.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
/**
 * @brief SignalPipeline 提供可配置的信号处理默认实现。
 */
class SignalPipeline final : public extension::ISignalPipeline {
 public:
  /**
   * @brief 构造信号处理流水线。
   * @param config 流水线运行配置。
   */
  explicit SignalPipeline(SignalPipelineConfig config = {});
  /**
   * @brief 析构信号处理流水线。
   */
  ~SignalPipeline() override;

  /**
   * @brief 执行单周期信号处理流程。
   * @param input_state 当前周期输入目标状态。
   * @param environment 当前环境服务。
   * @return 当前周期的信号处理输出。
   */
  extension::SignalCycleResult RunCycle(
      const model::TargetFeatureList& input_state,
      const extension::IEnvironmentService& environment) override;

  /**
   * @brief 获取上一周期生成的跟踪量测列表。
   * @return 与上一周期运行结果对应的跟踪量测集合。
   */
  std::vector<tracking::TrackMeasurement> GetLastTrackMeasurements() const;

  /**
   * @brief 获取上一周期的关联质量指标。
   * @return 上一周期缓存的关联质量指标。
   */
  extension::AssociationQualityMetrics GetLastAssociationQualityMetrics() const override;

  /**
   * @brief 注入关联阶段下一周期使用的轨迹种子。
   * @param seeds 由生命周期阶段导出的轨迹种子。
   */
  void SetAssociationSeeds(const std::vector<tracking::AssociationTrackSeed>& seeds);

  /**
   * @brief 清空外部注入的关联种子并回到无先验模式。
   */
  void ResetAssociationSeedModeToStateless();

  /**
   * @brief 按当前配置构造默认生命周期管理器。
   * @return 生命周期管理器实例。
   */
  std::unique_ptr<tracking::ITrackLifecycleManager> CreateAutoLifecycleManager() const;

  /**
   * @brief 更新平台姿态输入。
   * @param platform_attitude_deg 当前平台姿态。
   */
  void UpdatePlatformAttitude(
      const model::PlatformAttitudeDeg& platform_attitude_deg) override;

  /**
   * @brief 获取当前缓存的平台姿态。
   * @return 当前平台姿态。
   */
  model::PlatformAttitudeDeg GetPlatformAttitude() const override;

  /**
   * @brief 更新当前生效的控制真值。
   * @param control_profile 控制真值。
   */
  void SetControlProfile(const extension::control::RadarControlProfile& control_profile) override;

  /**
   * @brief 获取当前缓存的控制真值。
   * @return 当前控制真值。
   */
  extension::control::RadarControlProfile GetControlProfile() const override;

  /**
   * @brief 更新流水线运行配置。
   * @param config 新配置。
   */
  void UpdateConfig(SignalPipelineConfig config) override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_SIGNAL_PIPELINE_H_
