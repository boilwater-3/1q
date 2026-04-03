/**
 * @file InterceptPipeline.h
 * @brief 定义电子侦察默认流水线实现。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_PIPELINE_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_PIPELINE_H_

#include <cstdint>
#include <random>

#include "1q/electronic_surveillance_radar/pipeline/IMutableInterceptPipeline.h"
#include "electronic_surveillance_radar/pipeline/HypothesisAssociator.h"
#include "electronic_surveillance_radar/pipeline/KdTreeClusterer.h"
#include "electronic_surveillance_radar/pipeline/ObservationPipelineTypes.h"
#include "electronic_surveillance_radar/pipeline/ObservationPreprocessor.h"

namespace electronic_surveillance_radar {
namespace pipeline {

/**
 * @brief InterceptPipeline 是电子侦察流水线默认实现。
 */
class InterceptPipeline final : public IMutableInterceptPipeline {
 public:
  /**
   * @brief 构造默认流水线。
   * @param[in] config 流水线配置。
   * @param[in] runtime_config 会话运行态参数。
   */
  explicit InterceptPipeline(InterceptPipelineConfig config = {},
                             InterceptRuntimeConfig runtime_config = {});

  /**
   * @brief 更新流水线配置。
   * @param[in] config 新配置。
   */
  void UpdateConfig(InterceptPipelineConfig config) override;

  /**
   * @brief 更新运行态配置。
   * @param[in] runtime_config 新运行态配置。
   */
  void UpdateRuntimeConfig(InterceptRuntimeConfig runtime_config) override;

  /**
   * @brief 执行单周期流水线。
   * @param[in] input_state 当前周期输入。
   * @param[in] environment 环境服务。
   * @return 单周期输出。
   */
  InterceptCycleResult RunCycle(const core::context::EsrCycleInput& input_state,
                                const environment::IEsrEnvironmentService& environment) override;

 private:
  InterceptPipelineConfig config_{};
  InterceptRuntimeConfig runtime_config_{};
  internal::ObservationFeatureScales feature_scales_{};
  internal::ObservationPreprocessor preprocessor_{};
  internal::KdTreeClusterer clusterer_{};
  internal::HypothesisAssociator associator_{};
  std::mt19937 rng_{};
  std::uint64_t next_observation_id_{1U};
  std::uint64_t next_hypothesis_id_{1U};
};

}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_PIPELINE_H_
