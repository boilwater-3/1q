/**
 * @file InterceptPipeline.h
 * @brief 定义电子侦察默认流水线实现。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_PIPELINE_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_PIPELINE_H_

#include <cstdint>
#include <random>

#include "1q/electronic_surveillance_radar/pipeline/IInterceptPipeline.h"
#include "electronic_surveillance_radar/pipeline/HypothesisAssociator.h"
#include "electronic_surveillance_radar/pipeline/KdTreeClusterer.h"
#include "electronic_surveillance_radar/pipeline/ObservationPipelineTypes.h"
#include "electronic_surveillance_radar/pipeline/ObservationPreprocessor.h"

namespace electronic_surveillance_radar {
namespace pipeline {

/**
 * @brief InterceptRuntimeConfig 描述会话层注入的运行态参数。
 */
struct InterceptRuntimeConfig {
  bool sensor_enabled{true};              /**< 设备是否开启 */
  bool use_fixed_receiver_window{false};  /**< 是否启用固定接收频段窗口 */
  double receiver_lower_hz{0.0};          /**< 固定接收频段下限（单位：Hz） */
  double receiver_upper_hz{0.0};          /**< 固定接收频段上限（单位：Hz） */
  float integrated_receive_loss_db{0.0f}; /**< 系统综合接收损耗（单位：dB） */
  float antenna_mount_az_deg{0.0f};       /**< 天线方位安装角（单位：deg） */
  float antenna_mount_el_deg{0.0f};       /**< 天线俯仰安装角（单位：deg） */
  float scan_rate_hz{1.0f};               /**< 扫描数据率（单位：Hz） */
};

/**
 * @brief InterceptPipeline 是电子侦察流水线默认实现。
 */
class InterceptPipeline final : public IInterceptPipeline {
 public:
  /**
   * @brief 构造默认流水线。
   * @param[in] config 流水线配置。
   * @param[in] runtime_config 会话运行态参数。
   */
  explicit InterceptPipeline(InterceptPipelineConfig config = {},
                             InterceptRuntimeConfig runtime_config = {});

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
