/**
 * @file InterceptPostProcessingExecutor.h
 * @brief 定义电子侦察后处理执行器。
 *
 * InterceptPostProcessingExecutor 封装预处理、特征编码、聚类、
 * 簇摘要、假设关联与真值评估，
 * 从 InterceptPipeline::RunCycle 的后处理阶段提取而来。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_POST_PROCESSING_EXECUTOR_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_POST_PROCESSING_EXECUTOR_H_

#include <cstdint>
#include <vector>

#include "1q/electronic_surveillance_radar/extension/IEsrContext.h"
#include "1q/electronic_surveillance_radar/extension/InterceptPipelineTypes.h"
#include "electronic_surveillance_radar/pipeline/HypothesisAssociator.h"
#include "electronic_surveillance_radar/pipeline/KdTreeClusterer.h"
#include "electronic_surveillance_radar/pipeline/ObservationPipelineTypes.h"
#include "electronic_surveillance_radar/pipeline/ObservationPreprocessor.h"

namespace electronic_surveillance_radar {
namespace pipeline {

/**
 * @brief InterceptPostProcessingExecutor 执行后处理阶段。
 *
 * 职责：
 *   - 原始观测预处理
 *   - 观测特征编码
 *   - KD-tree 聚类
 *   - 簇摘要构建（含频谱分析）
 *   - 假设关联更新
 *   - 真值评估关联构建
 */
class InterceptPostProcessingExecutor {
 public:
  /**
   * @brief 执行后处理。
   * @param[in] raw_records 检测阶段产出的原始观测记录。
   * @param[in] ctx 周期上下文。
   * @param[in,out] preprocessor 预处理器。
   * @param[in,out] clusterer 聚类器。
   * @param[in,out] associator 假设关联器。
   * @param[in] feature_scales 特征尺度。
   * @param[in,out] next_hypothesis_id 假设 ID 分配器。
   * @return 单周期流水线输出。
   */
  extension::InterceptPipelineResult Execute(const std::vector<RawObservationRecord>& raw_records,
                                             const extension::IEsrContext& ctx,
                                             ObservationPreprocessor& preprocessor,
                                             KdTreeClusterer& clusterer,
                                             HypothesisAssociator& associator,
                                             const ObservationFeatureScales& feature_scales,
                                             std::uint64_t& next_hypothesis_id);
};

}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_POST_PROCESSING_EXECUTOR_H_
