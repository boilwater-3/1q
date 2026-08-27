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

  /**
   * @brief 同上，但可选抑制验收日志写出。
   * @param[in] write_acceptance false 时不写推演验收行（如精度评估层用真值状态做
   *            对照推演时，避免真值行混入产品验收日志、避免双推进关机状态机）。
   */
  std::vector<TargetInferenceResult> Infer(const std::vector<InferenceTrackState>& tracks,
                                           bool write_acceptance) const;

  /**
   * @brief 位置 1-σ 敏度传播到指定时刻偏移（引擎敏度框架的公开静态面）。
   * @param[in] track 航迹状态（须携带协方差，否则返回 0）。
   * @param[in] time_offset_sec 目标时刻相对输入时刻的偏移（可为负 = 回推到过去）。
   * @param[in] config 推演配置（单源动力学参数）。
   * @return 位置 1-σ（m）；6 扰动对角简化口径（Σ_j Σ_i h_ij²·P_jj 开方），与落点
   *         1-σ 同框架。推演不有限时返回 0。
   * @note 供验收旁路把关机点协方差传播到关机时刻（评审 2026-08-26 条10）。
   */
  static double PositionSigmaAt(const InferenceTrackState& track, double time_offset_sec,
                                const TargetInferenceConfig& config);

  /**
   * @brief 标称弹道位置传播到指定时刻偏移（RK4 前向/后向，与航路点同动力学）。
   * @param[in] track 航迹状态（不要求协方差）。
   * @param[in] time_offset_sec 目标时刻相对输入时刻的偏移（可为负 = 回推到过去）。
   * @param[in] config 推演配置（单源动力学参数）。
   * @param[out] out_lla 输出传播位置（度制 LLA）。
   * @return 传播有限且 ECEF→LLA 换算成功时为真。
   * @note 供验收旁路逐秒采样预测点列表（评审 2026-08-27 条2：10 个 LLA 预测点
   *       泛式代表未来 10s 轨迹）；与 PositionSigmaAt 同款步进（dt 可为负）。
   */
  static bool PositionAt(const InferenceTrackState& track, double time_offset_sec,
                         const TargetInferenceConfig& config,
                         oneq::coordinate::LlaPositionDegM* out_lla);

 private:
  TargetInferenceConfig config_;
};

}  // namespace target_inference

#endif  // ONEQ_TARGET_INFERENCE_TARGET_INFERENCE_ENGINE_H_
