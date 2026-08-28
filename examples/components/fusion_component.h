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
#include "core/events.h"
#include "core/signals.h"

namespace component_attachment {

/**
 * @brief 融合组件：多源探测聚合 + 融合态势更新；双星场景兼挂评估会话。
 */
class FusionComponent : public Component {
 public:
  /**
   * @brief 仅融合引擎（无精度评估叠加）。
   * @param[in] engine 调用方构造后移入的融合引擎（示例：main 读取场景 JSON 的 fusion 段，
   *            `new FusionEngine(scene.config.fusion)`）。
   */
  explicit FusionComponent(std::unique_ptr<fusion::FusionEngine> engine);
  /**
   * @brief 地面站：融合引擎 + 评估会话（双星探测帧在本组件内适配后 Update）。
   * @param[in] engine 同单参构造（场景 JSON fusion 段 → FusionEngine）。
   * @param[in] evaluation 调用方构造后移入的评估会话（示例：main 用场景 JSON session_config
   *            `new PrecisionEvaluationSession(scene.config)`）。
   * @param[in] satellite_a_source_id 主星融合源通道（示例：scene.config.satellite_a_source_id）。
   * @param[in] satellite_b_source_id 辅星融合源通道（示例：scene.config.satellite_b_source_id；
   *            与主星相同时组件内自动 +100）。
   */
  FusionComponent(std::unique_ptr<fusion::FusionEngine> engine,
                  std::unique_ptr<precision_evaluation::PrecisionEvaluationSession> evaluation,
                  std::uint32_t satellite_a_source_id, std::uint32_t satellite_b_source_id);
  ~FusionComponent() override = default;

  FusionComponent(const FusionComponent&) = delete;
  FusionComponent& operator=(const FusionComponent&) = delete;

  const char* Name() const override { return "Fusion"; }
  void Step(World& world, double dt_sec) override;

  /** @brief 新周期开始前清空消息收件箱并确保信号已连接（须在 World::Step 之前调用）。 */
  void BeginCycle(World& world, std::uint64_t cycle);

  /** @brief 当前融合目标态势（按 key 升序；app 层汇总读，组件间走事件）。 */
  const std::vector<fusion::FusedTarget>& targets() const { return targets_; }

  /**
   * @brief 写入本周期评估对照输入（每周期 World::Step 前由 main 注入）。
   * @param[in] ephemeris 场景 JSON ephemeris 段（双星星历，挂载后不变）。
   * @param[in] truth 场景 JSON truth 段（main 每周期按 dt 推进弹道后传入）。
   */
  void SetEvaluationInputs(const precision_evaluation::DualSatEphemerisInput& ephemeris,
                           const std::vector<precision_evaluation::EvaluationTruthTarget>& truth);

  /** @brief 最近一周期评估样本（未挂评估会话时为空）。 */
  const precision_evaluation::PrecisionEvaluationCycleResult& last_evaluation() const {
    return last_evaluation_;
  }

  /** @brief 全程评估报告（未挂评估会话时默认空报告）。 */
  precision_evaluation::PrecisionEvaluationReport SummarizeEvaluation() const;

 private:
  void EnsureSignalConnections(World& world);
  void OnDetectionBatchSubmitted(const DetectionBatchSubmittedEvent& event);

  std::unique_ptr<fusion::FusionEngine> engine_;
  std::vector<fusion::FusedTarget> targets_{};

  std::uint64_t inbox_cycle_{0U};
  std::vector<FusionDetectionSample> detection_inbox_{};
  boost::signals2::scoped_connection detection_batch_connection_{};

  std::unique_ptr<precision_evaluation::PrecisionEvaluationSession> evaluation_;
  std::uint32_t satellite_a_source_id_{4U}; /**< 主星融合源通道 */
  std::uint32_t satellite_b_source_id_{104U}; /**< 辅星融合源通道 */
  precision_evaluation::DualSatEphemerisInput evaluation_ephemeris_{};
  std::vector<precision_evaluation::EvaluationTruthTarget> evaluation_truth_{};
  precision_evaluation::PrecisionEvaluationCycleResult last_evaluation_{};
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_FUSION_COMPONENT_H_
