/**
 * @file ground_station_fusion_component.h
 * @brief 消息驱动地面站融合组件（订阅探测事件 → FusionEngine::Update）。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_GROUND_STATION_FUSION_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_GROUND_STATION_FUSION_COMPONENT_H_

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

class World;

/**
 * @brief 地面站融合枢纽：消息收件箱 + 融合引擎（可选双星精度评估叠加）。
 */
class GroundStationFusionComponent : public Component {
 public:
  explicit GroundStationFusionComponent(std::unique_ptr<fusion::FusionEngine> engine);
  /**
   * @brief 融合引擎 + 精度评估会话。
   * @param[in] evaluation_source_ids 评估叠加用的两颗星源通道（库内交会 API 仍是双星）；
   *            融合本身按 inbox 里本周期全部卫星处理，不限两颗。
   */
  GroundStationFusionComponent(
      std::unique_ptr<fusion::FusionEngine> engine,
      std::unique_ptr<precision_evaluation::PrecisionEvaluationSession> evaluation,
      std::vector<std::uint32_t> evaluation_source_ids);
  ~GroundStationFusionComponent() override = default;

  GroundStationFusionComponent(const GroundStationFusionComponent&) = delete;
  GroundStationFusionComponent& operator=(const GroundStationFusionComponent&) = delete;

  const char* Name() const override { return "GroundStationFusion"; }
  void Step(World& world, double dt_sec) override;

  /** @brief 新周期开始前清空收件箱并确保信号已连接（须在 World::Step 之前调用）。 */
  void BeginCycle(World& world, std::uint64_t cycle);

  void SetEvaluationInputs(const precision_evaluation::DualSatEphemerisInput& ephemeris,
                           const std::vector<precision_evaluation::EvaluationTruthTarget>& truth);

  const std::vector<fusion::FusedTarget>& targets() const { return targets_; }

  const precision_evaluation::PrecisionEvaluationCycleResult& last_evaluation() const {
    return last_evaluation_;
  }

  precision_evaluation::PrecisionEvaluationReport SummarizeEvaluation() const;

 private:
  void EnsureSignalConnections(World& world);
  void OnDetectionBatchSubmitted(const DetectionBatchSubmittedEvent& event);
  void OnSbirsFrameSubmitted(const SbirsFrameSubmittedEvent& event);

  std::unique_ptr<fusion::FusionEngine> engine_;
  std::vector<fusion::FusedTarget> targets_{};

  std::uint64_t inbox_cycle_{0U};
  std::vector<FusionDetectionSample> detection_inbox_{};
  std::vector<SbirsFrameSubmittedEvent> sbirs_inbox_{};

  std::unique_ptr<precision_evaluation::PrecisionEvaluationSession> evaluation_;
  std::vector<std::uint32_t> evaluation_source_ids_{};
  precision_evaluation::DualSatEphemerisInput evaluation_ephemeris_{};
  std::vector<precision_evaluation::EvaluationTruthTarget> evaluation_truth_{};
  precision_evaluation::PrecisionEvaluationCycleResult last_evaluation_{};

  boost::signals2::scoped_connection detection_batch_connection_{};
  boost::signals2::scoped_connection sbirs_frame_connection_{};
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_GROUND_STATION_FUSION_COMPONENT_H_
