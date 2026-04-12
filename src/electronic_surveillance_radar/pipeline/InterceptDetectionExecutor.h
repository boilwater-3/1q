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
#include <vector>

#include "1q/electronic_surveillance_radar/extension/IEsrContext.h"
#include "1q/electronic_surveillance_radar/extension/InterceptPipelineTypes.h"
#include "electronic_surveillance_radar/intercept/ScanPatternGenerator.h"
#include "electronic_surveillance_radar/pipeline/ObservationPipelineTypes.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace internal {

/**
 * @brief InterceptDetectionOutput 描述检测阶段输出。
 */
struct InterceptDetectionOutput {
  std::vector<RawObservationRecord> raw_records;        /**< 检测产出的原始观测记录 */
  std::vector<intercept::BeamPointingDeg> scan_pattern; /**< 当前扫描图 */
};

/**
 * @brief InterceptDetectionExecutor 执行截获检测阶段。
 *
 * 职责：
 *   - 生成当前周期扫描图
 *   - 遍历场景辐射源，执行截获门限判定
 *   - 施加欺骗扰动与虚警注入
 *   - 输出带真值标注的原始观测记录
 */
class InterceptDetectionExecutor {
 public:
  /**
   * @brief 执行截获检测。
   * @param[in] ctx 周期上下文。
   * @param[in,out] rng 随机引擎。
   * @param[in,out] next_observation_id 观测 ID 分配器。
   * @return 检测阶段输出。
   */
  InterceptDetectionOutput Execute(const extension::IEsrContext& ctx, std::mt19937& rng,
                                   std::uint64_t& next_observation_id);
};

}  // namespace internal
}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_DETECTION_EXECUTOR_H_
