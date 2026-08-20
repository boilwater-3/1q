/**
 * @file PrecisionEvaluationSession.h
 * @brief 定义精度评估编排会话：双星 SBIRS + 融合 + 推演的真值对照误差提取（需求 3.2.1.6.3）。
 */

#ifndef ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_SESSION_H_
#define ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_SESSION_H_

#include <cstdint>
#include <memory>
#include <vector>

#include "1q/api.hpp"
#include "1q/precision_evaluation/PrecisionEvaluationConfig.h"
#include "1q/precision_evaluation/PrecisionEvaluationTypes.h"

namespace precision_evaluation {

/**
 * @brief 精度评估编排会话（评估层入口）。
 * @details 每周期 Step：同目标场景、同 GMST 驱动两个独立 SBIRS 会话（主/辅星）→
 *          ① 红外定位角度误差（各星输出角 vs 真值角）；② 双星双视线交会位置误差
 *          （同周期两星同目标均检出时）；③ 两星检测（经归属层 target_id 关联 + ENU
 *          适配）进融合引擎，取逐航迹运动学估计 → 速度误差；④ 按 inference_interval
 *          周期以估计状态与真值状态分别喂推演引擎 → 落点/发射点预测误差（真值关键点
 *          每目标缓存一次）。Summarize 聚合五指标（mean/RMSE/P95/max）并求 AHP 权重
 *          与综合得分。
 * @note 内部强制 `fusion.enable_track_filtering = true`（速度/位置误差样本依赖逐航迹
 *       滤波，默认关的融合无运动学估计）。评估只消费各层公开产品与场景真值，不回写
 *       任何被评估模块；`[PrecisionEval]` 日志事件由编译期开关
 *       `ONEQ_ENABLE_PRECISION_EVALUATION_LOG`（默认 OFF）门控。
 * @note 落点/发射点误差口径 = 估计状态与真值状态经**同一推演引擎**的关键点之差，
 *       衡量状态误差传播，不含弹道模型自身偏差。
 */
class ONEQ_API PrecisionEvaluationSession {
 public:
  /**
   * @brief 构造评估会话（双星 SBIRS + 融合 + 推演按配置装配）。
   * @param[in] config 评估聚合配置。
   */
  explicit PrecisionEvaluationSession(const PrecisionEvaluationConfig& config = {});

  ~PrecisionEvaluationSession();

  PrecisionEvaluationSession(PrecisionEvaluationSession&&) noexcept;
  PrecisionEvaluationSession& operator=(PrecisionEvaluationSession&&) noexcept;

  PrecisionEvaluationSession(const PrecisionEvaluationSession&) = delete;
  PrecisionEvaluationSession& operator=(const PrecisionEvaluationSession&) = delete;

  /**
   * @brief 执行一个评估周期（驱动双星传感器、融合与按间隔的推演）。
   * @param[in] cycle_index 周期号（调用方单调递增）
   * @param[in] dt_sec 步长（s）
   * @param[in] utc_julian_day UTC 儒略日（GMST 计算）
   * @param[in] ephemeris 双星星历（ECEF 位置/速度 + ECI 姿态）
   * @param[in] truth_targets 本周期真值目标列表（键须与各层产品可关联）
   * @return 本周期四类误差样本
   */
  PrecisionEvaluationCycleResult Step(std::uint32_t cycle_index, float dt_sec,
                                      double utc_julian_day,
                                      const DualSatEphemerisInput& ephemeris,
                                      const std::vector<EvaluationTruthTarget>& truth_targets);

  /**
   * @brief 聚合全程误差样本并求 AHP 综合评分（不改变会话状态，可重复调用）。
   * @return 五指标汇总 + AHP 权重/一致性 + 综合得分；AHP 矩阵非法时 ahp_valid=false
   *         且综合分保持 0（不静默退化为等权）；无样本指标按 0 分计入综合
   *         （零证据=零分，不因空序列 rmse=0 而得满分）。
   */
  PrecisionEvaluationReport Summarize() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace precision_evaluation

#endif  // ONEQ_PRECISION_EVALUATION_PRECISION_EVALUATION_SESSION_H_
