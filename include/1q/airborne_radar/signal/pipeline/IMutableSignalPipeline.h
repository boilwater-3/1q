/**
 * @file IMutableSignalPipeline.h
 * @brief 定义可变信号流水线接口，供 Session 外部装配注入。
 */

#ifndef AIRBORNE_RADAR_SIGNAL_PIPELINE_I_MUTABLE_SIGNAL_PIPELINE_H_
#define AIRBORNE_RADAR_SIGNAL_PIPELINE_I_MUTABLE_SIGNAL_PIPELINE_H_

#include "1q/airborne_radar/config/SignalPipelineConfig.h"
#include "1q/airborne_radar/signal/pipeline/ISignalPipeline.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

/**
 * @brief 在 ISignalPipeline 基础上补充运行期配置更新与指标读取能力。
 */
class ONEQ_API IMutableSignalPipeline : public ISignalPipeline {
 public:
  ~IMutableSignalPipeline() override = default;

  /**
   * @brief 更新流水线运行配置。
   * @param config 新配置。
   */
  virtual void UpdateConfig(common::config::SignalPipelineConfig config) = 0;

  /**
   * @brief 获取上一周期关联质量指标。
   * @return 上一周期缓存的关联质量指标。
   */
  virtual AssociationQualityMetrics GetLastAssociationQualityMetrics() const = 0;
};

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SIGNAL_PIPELINE_I_MUTABLE_SIGNAL_PIPELINE_H_
