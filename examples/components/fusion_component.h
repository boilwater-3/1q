/**
 * @file fusion_component.h
 * @brief 自定义实体-组件示例：融合组件（地面站枢纽）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_FUSION_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_FUSION_COMPONENT_H_

#include <cstdint>
#include <memory>
#include <vector>

#include "1q/fusion/FusionEngine.h"
#include "1q/fusion/FusedTarget.h"
#include "1q/precision_evaluation/PrecisionEvaluationSession.h"
#include "1q/precision_evaluation/PrecisionEvaluationTypes.h"
#include "core/component.h"

namespace component_attachment {

/**
 * @brief 融合组件：多源探测聚合 + 融合态势更新；双星场景兼挂评估会话。
 */
class FusionComponent : public Component {
 public:
  explicit FusionComponent(std::unique_ptr<fusion::FusionEngine> engine);
  /**
   * @brief 地面站：融合引擎 + 评估会话（双星探测帧在本组件内适配后 Update）。
   */
  FusionComponent(std::unique_ptr<fusion::FusionEngine> engine,
                  std::unique_ptr<precision_evaluation::PrecisionEvaluationSession> evaluation,
                  std::uint32_t satellite_a_source_id, std::uint32_t satellite_b_source_id);
  ~FusionComponent() override = default;

  FusionComponent(const FusionComponent&) = delete;
  FusionComponent& operator=(const FusionComponent&) = delete;

  const char* Name() const override { return "Fusion"; }
  void Step(World& world, double dt_sec) override;

  /** @brief 当前融合目标态势（按 key 升序；app 层汇总读，组件间走事件）。 */
  const std::vector<fusion::FusedTarget>& targets() const { return targets_; }

  /** @brief 本周期评估对照用的星历与真值（地面站场景在 World::Step 前写入）。 */
  void SetEvaluationInputs(const precision_evaluation::DualSatEphemerisInput& ephemeris,
                           const std::vector<precision_evaluation::EvaluationTruthTarget>& truth);

  /** @brief 最近一周期评估样本（未挂评估会话时为空）。 */
  const precision_evaluation::PrecisionEvaluationCycleResult& last_evaluation() const {
    return last_evaluation_;
  }

  /** @brief 全程评估报告（未挂评估会话时默认空报告）。 */
  precision_evaluation::PrecisionEvaluationReport SummarizeEvaluation() const;

 private:
  std::unique_ptr<fusion::FusionEngine> engine_;
  std::vector<fusion::FusedTarget> targets_{};

  std::unique_ptr<precision_evaluation::PrecisionEvaluationSession> evaluation_;
  std::uint32_t satellite_a_source_id_{4U}; /**< 主星融合源通道 */
  std::uint32_t satellite_b_source_id_{104U}; /**< 辅星融合源通道 */
  precision_evaluation::DualSatEphemerisInput evaluation_ephemeris_{};
  std::vector<precision_evaluation::EvaluationTruthTarget> evaluation_truth_{};
  precision_evaluation::PrecisionEvaluationCycleResult last_evaluation_{};
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_FUSION_COMPONENT_H_
