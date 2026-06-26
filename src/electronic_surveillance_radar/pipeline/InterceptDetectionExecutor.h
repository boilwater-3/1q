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

#include <utility>

#include "electronic_surveillance_radar/pipeline/IEsrContext.h"
#include "electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
#include "common/timing/TimingRegimeModel.h"
#include "electronic_surveillance_radar/intercept/AngleErrorModel.h"
#include "electronic_surveillance_radar/intercept/ScanPatternGenerator.h"
#include "electronic_surveillance_radar/pipeline/ObservationPipelineTypes.h"

namespace electronic_surveillance_radar {
namespace pipeline {

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

 private:
  /**
   * @brief 单个辐射源的截获判定与观测记录生成。
   * @param emitter 当前辐射源真值。
   * @param active_beam 当前周期激活波束。
   * @param receiver_window 当前接收频段窗口。
   * @param receive_loss_scale 综合接收损耗线性比例。
   * @param angle_error_config 测角误差模型配置。
   * @param base_statistical_detection_params 统计检测基线参数。
   * @param ctx 周期上下文（只读部分）。
   * @param rng 随机引擎。
   * @param next_observation_id 观测 ID 分配器。
   * @param raw_records 产出观测记录列表。
   */
  void ProcessSingleEmitter(
      const session::EsrSceneEmitter& emitter,
      const intercept::BeamPointingDeg& active_beam,
      const std::pair<double, double>& receiver_window,
      double receive_loss_scale,
      const intercept::AngleErrorModelConfig& angle_error_config,
      const oneq::internal::timing::StatisticalDetectionParams& base_statistical_detection_params,
      const extension::IEsrContext& ctx,
      std::mt19937& rng,
      std::uint64_t& next_observation_id,
      std::vector<RawObservationRecord>& raw_records) const;
};

}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_INTERCEPT_DETECTION_EXECUTOR_H_
