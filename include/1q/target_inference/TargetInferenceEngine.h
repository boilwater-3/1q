/**
 * @file TargetInferenceEngine.h
 * @brief 定义目标推演引擎（无状态纯函数算法面：航迹状态 → 带误差预算的推演结论）。
 */

#ifndef ONEQ_TARGET_INFERENCE_TARGET_INFERENCE_ENGINE_H_
#define ONEQ_TARGET_INFERENCE_TARGET_INFERENCE_ENGINE_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/target_inference/InferenceResult.h"
#include "1q/target_inference/InferenceTrackState.h"
#include "1q/target_inference/TargetInferenceConfig.h"

namespace target_inference {

/**
 * @brief 目标推演引擎（JDL 推演层算法面，先例：threat_assessment）。
 * @details 每次调用对每条输入航迹独立执行：弹道前向外推（轨迹/落点）+ 反向积分
 *          （发射点）+ 类型概率融合（运动学先验 + 外部证据）。引擎无跨调用状态，
 *          同输入同输出（确定性）；逐航迹独立，无航迹间交互。
 * @note 产品必须消费误差预算（协方差/σ），不得只读点估计（分层契约规则 6）。
 */
class ONEQ_API TargetInferenceEngine {
 public:
  /**
   * @brief 构造推演引擎。
   * @param[in] config 推演配置。
   */
  explicit TargetInferenceEngine(const TargetInferenceConfig& config = {});

  ~TargetInferenceEngine();

  TargetInferenceEngine(const TargetInferenceEngine&) = delete;
  TargetInferenceEngine& operator=(const TargetInferenceEngine&) = delete;

  /**
   * @brief 对一批航迹状态执行推演。
   * @param[in] tracks 航迹输入帧列表（调用方按融合航迹组装）。
   * @return 与输入顺序一致的推演结果列表（确定性输出）。
   */
  std::vector<TargetInferenceResult> Infer(const std::vector<InferenceTrackState>& tracks) const;

 private:
  TargetInferenceConfig config_;
};

}  // namespace target_inference

#endif  // ONEQ_TARGET_INFERENCE_TARGET_INFERENCE_ENGINE_H_
