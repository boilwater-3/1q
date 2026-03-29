/**
 * @file RadarSession.h
 * @brief 定义面向外部接入的高层机载雷达会话门面。
 */

#ifndef AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_H_
#define AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_H_

#include <memory>
#include <vector>

#include "1q/airborne_radar/common/RadarCommand.h"
#include "1q/airborne_radar/common/RadarControlProfile.h"
#include "1q/airborne_radar/common/TrackOutputFrame.h"
#include "1q/airborne_radar/core/context/RadarCycleInput.h"
#include "1q/airborne_radar/core/session/RadarCycleResult.h"
#include "1q/airborne_radar/environment/EnvironmentTypes.h"
#include "1q/airborne_radar/signal/pipeline/SignalPipelineTypes.h"
#include "1q/api.hpp"

namespace airborne_radar {
namespace core {
namespace session {

/**
 * @brief RadarSessionConfig 描述 RadarSession 的默认装配配置。
 */
struct ONEQ_API RadarSessionConfig {
  signal::pipeline::SignalPipelineConfig signal_pipeline_config{}; /**< 信号流水线配置 */
  environment::EnvironmentModelConfig environment_model_config{};  /**< 环境模型配置 */
  float jamming_detection_threshold_db{6.0f};                      /**< 干扰判定阈值（单位：dB） */
};

/**
 * @brief RadarSession 提供"一步一帧"的外部接入门面。
 */
class ONEQ_API RadarSession {
 public:
  /** @brief 使用默认链路构造会话 */
  explicit RadarSession(const RadarSessionConfig& config = {});
  ~RadarSession();

  RadarSession(const RadarSession&) = delete;
  RadarSession& operator=(const RadarSession&) = delete;

  /**
   * @brief 执行一个不显式切场景的处理周期。
   * @param input 当前周期输入。
   * @return 当前周期生成的轨迹输出帧拷贝。
   * @note 输入容错：`dt_sec ≤ 0` 或其他非法输入时，函数不抛异常，
   *       会以尽力而为出发返回上一周期有效状态，输出帧中
   *       `published_track_count` 可能为零。
   */
  common::TrackOutputFrame Step(const context::RadarCycleInput& input);

  /**
   * @brief 先更新待生效场景，再执行当前周期。
   * @param input 当前周期输入。
   * @param scene_state 当前 step 之前要提交的待生效场景。
   * @return 当前周期生成的轨迹输出帧拷贝。
   * @note 输入容错：`dt_sec ≤ 0` 或其他非法输入时，函数不抛异常，
   *       会以尽力而为出发返回上一周期有效状态，输出帧中
   *       `published_track_count` 可能为零。
   */
  common::TrackOutputFrame Step(const context::RadarCycleInput& input,
                                const environment::EnvironmentSceneState& scene_state);

  /**
   * @brief 执行一个不显式切场景的处理周期，并返回聚合结果。
   * @param input 当前周期输入。
   * @return 当前周期聚合结果。
   * @note 输入容错：行为与 `Step()` 一致。
   */
  RadarCycleResult StepWithResult(const context::RadarCycleInput& input);

  /**
   * @brief 先更新待生效场景，再执行当前周期，并返回聚合结果。
   * @param input 当前周期输入。
   * @param scene_state 当前 step 之前要提交的待生效场景。
   * @return 当前周期聚合结果。
   */
  RadarCycleResult StepWithResult(const context::RadarCycleInput& input,
                                  const environment::EnvironmentSceneState& scene_state);

  /** @brief 获取当前周期已提交的控制指令。
   * @return 当前周期已提交的控制指令列表引用。
   */
  const std::vector<common::RadarCommand>& GetSubmittedCommands() const;

  /**
   * @brief 判断是否已经保存过最新控制真值。
   * @return 若已持有最近一次控制真值则返回 true。
   */
  bool HasLatestControlProfile() const;

  /**
   * @brief 获取最近一次控制真值。
   * @return 最近一次控制真值引用。
   */
  const common::RadarControlProfile& GetLatestControlProfile() const;

  /**
   * @brief 获取最近一次关联质量观测指标。
   * @return 最近一次关联质量观测指标。
   */
  signal::pipeline::AssociationQualityMetrics GetLastAssociationQualityMetrics() const;

  /**
   * @brief 更新信号流水线配置。
   * @param[in] config 信号流水线配置。
   */
  void UpdateSignalPipelineConfig(const signal::pipeline::SignalPipelineConfig& config);

  /**
   * @brief 更新环境模型配置。
   * @param[in] config 环境模型配置。
   */
  void UpdateEnvironmentModelConfig(const environment::EnvironmentModelConfig& config);

  /**
   * @brief 更新干扰判定阈值。
   * @param[in] threshold_db 干扰判定阈值（单位：dB）。
   */
  void SetJammingDetectionThresholdDb(float threshold_db);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace session
}  // namespace core
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_CORE_SESSION_RADAR_SESSION_H_
