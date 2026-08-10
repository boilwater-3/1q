/**
 * @file ThreatEvaluator.h
 * @brief 定义归一化加权和威胁评估器。
 */

#ifndef ONEQ_THREAT_ASSESSMENT_THREAT_EVALUATOR_H_
#define ONEQ_THREAT_ASSESSMENT_THREAT_EVALUATOR_H_

#include <vector>

#include "1q/api.hpp"
#include "1q/threat_assessment/ThreatEvaluationInput.h"
#include "1q/threat_assessment/ThreatEvaluatorConfig.h"
#include "1q/threat_assessment/ThreatResult.h"

namespace threat_assessment {

/**
 * @brief 目标威胁评估器（归一化加权和 MADM，纯函数式无跨周期状态）。
 * @details 每个输入目标独立计算：六属性（距离/速度/加速度/RCS/类型概率/融合置信度）
 *          分段线性归一化到 [0,1] → 按配置权重加权求和 → 钳制 [0,1] →
 *          按配置阈值映射威胁等级。输出顺序与输入一致，同输入同输出（确定性）。
 *          跨周期记忆（历史/平滑）由调用方组装进输入帧，评估器不持有状态。
 */
class ONEQ_API ThreatEvaluator {
 public:
  /**
   * @brief 构造威胁评估器。
   * @param[in] config 评估配置（权重/归一化断点/等级阈值）。
   */
  explicit ThreatEvaluator(const ThreatEvaluatorConfig& config);

  /**
   * @brief 批量评估目标威胁。
   * @param[in] inputs 目标输入帧列表（可为空）。
   * @return 与输入顺序一致的威胁评估结果列表；输入为空时输出为空。
   */
  std::vector<ThreatResult> Evaluate(
      const std::vector<ThreatEvaluationInput>& inputs) const;

 private:
  ThreatEvaluatorConfig config_;
};

}  // namespace threat_assessment

#endif  // ONEQ_THREAT_ASSESSMENT_THREAT_EVALUATOR_H_
