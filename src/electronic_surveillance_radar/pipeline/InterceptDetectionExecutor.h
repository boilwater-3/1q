/**
 * @file InterceptDetectionExecutor.h
 * @brief 定义电子侦察截获检测执行器。
 *
 * InterceptDetectionExecutor 封装扫描图生成与逐辐射源截获检测循环，
 * 从 InterceptPipeline::RunCycle 的检测阶段提取而来。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_DETECTION_EXECUTOR_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_DETECTION_EXECUTOR_H_

#include <cstdint>
#include <random>
#include <utility>
#include <vector>

#include "common/timing/TimingRegimeModel.h"
#include "electronic_surveillance_radar/pipeline/AngleErrorModel.h"
#include "electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
#include "electronic_surveillance_radar/pipeline/MutableEsrContext.h"
#include "electronic_surveillance_radar/pipeline/ObservationPipelineTypes.h"
#include "electronic_surveillance_radar/pipeline/ScanPatternGenerator.h"

namespace electronic_surveillance_radar {
namespace pipeline {

/**
 * @brief InterceptDetectionOutput 描述检测阶段输出。
 */
struct InterceptDetectionOutput {
  std::vector<RawObservationRecord> raw_records;        /**< 检测产出的原始观测记录 */
  std::vector<intercept::BeamPointingDeg> scan_pattern; /**< 当前扫描图 */
  double receiver_center_frequency_hz{0.0};             /**< 当前接收中心频率（单位：Hz）。 */
  double receiver_bandwidth_hz{0.0};                    /**< 当前接收带宽（单位：Hz）。 */
  bool receiver_saturated{false};                       /**< 是否发生接收机饱和。 */
  bool rf_v2_rejected{false}; /**< RF v2 链路无法原子求解，调用方必须回滚周期状态。 */
};

/**
 * @brief InterceptDetectionExecutor 执行截获检测阶段。
 *
 * 职责：
 *   - 生成当前周期扫描图
 *   - 对冻结 RF v2 发射帧求解接收链和截获门限
 *   - 输出去真值化的原始观测记录
 */
class InterceptDetectionExecutor {
 public:
  /**
   * @brief 执行截获检测。
   * @param[in] ctx 周期上下文。
   * @param[in,out] rng 随机引擎。
   * @param[in,out] next_observation_id 观测 ID 分配器。
   * @param[in,out] scan_phase_cycles 归一化完整扫描图循环相位。
   * @return 检测阶段输出。
   */
  InterceptDetectionOutput Execute(const MutableEsrContext& ctx, std::mt19937& rng,
                                   std::uint64_t& next_observation_id, double* scan_phase_cycles,
                                   std::uint64_t completed_receive_cycles);

 private:
  /** @brief 使用统一 RF v2 发射帧执行接收、信道竞争和去真值化量测。 */
  bool ProcessRfV2Frame(
      const MutableEsrContext& ctx, const intercept::BeamPointingDeg& active_beam,
      const std::pair<double, double>& receiver_window,
      const intercept::AngleErrorModelConfig& angle_error_config,
      const oneq::common::timing::StatisticalDetectionParams& base_statistical_detection_params,
      std::mt19937& rng, std::uint64_t& next_observation_id,
      InterceptDetectionOutput* output) const;
};

}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_DETECTION_EXECUTOR_H_
