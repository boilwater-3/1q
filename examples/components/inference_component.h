/**
 * @file inference_component.h
 * @brief 自定义实体-组件示例：目标推演组件。
 */

#ifndef EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_INFERENCE_COMPONENT_H_
#define EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_INFERENCE_COMPONENT_H_

#include <array>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "1q/target_inference/InferenceResult.h"
#include "1q/target_inference/TargetInferenceConfig.h"
#include "1q/target_inference/TargetInferenceEngine.h"
#include "core/component.h"
#include "core/signals.h"

namespace component_attachment {

/**
 * @brief 目标推演组件：融合运动学估计 → 轨迹/发射点/落点/类型推演。
 *
 * 输入订阅融合态势事件（on_fusion_updated，含运动学估计展平字段）并逐目标
 * 缓存；Step 用本周期新鲜缓存重建库输入——不调用融合组件方法（集成方同走
 * 事件机制）。
 *
 * @note 与库内算法面的边界：本组件只做"订阅 + 重建 + 日志化"，推演逻辑
 *       全在 target_inference 模块。
 */
class InferenceComponent : public Component {
 public:
  explicit InferenceComponent(
      const target_inference::TargetInferenceConfig& config =
          target_inference::TargetInferenceConfig{});
  ~InferenceComponent() override = default;

  InferenceComponent(const InferenceComponent&) = delete;
  InferenceComponent& operator=(const InferenceComponent&) = delete;

  const char* Name() const override { return "TargetInference"; }
  void Step(World& world, double dt_sec) override;

  /** @brief 当前推演结果（与输入航迹同序）。 */
  const std::vector<target_inference::TargetInferenceResult>& results() const {
    return results_;
  }

 private:
  /// 融合态势事件缓存条目（事件展平字段原样保存；Step 重建库输入）。
  struct KinematicSnapshot {
    std::uint64_t cycle{0U};
    bool has_kinematic_estimate{false};
    double latitude_deg{0.0};
    double longitude_deg{0.0};
    double altitude_m{0.0};
    std::array<double, 3U> velocity_ecef_m_per_s{{0.0, 0.0, 0.0}};
    std::array<double, 36U> covariance_ecef{};
  };

  /// 融合态势事件回调（逐目标缓存；周期号供 Step 新鲜度过滤）。
  void OnFusionUpdated(const FusionUpdatedEvent& event);

  target_inference::TargetInferenceEngine engine_; /**< 推演引擎（纯函数式）。 */
  std::unordered_map<std::uint64_t, KinematicSnapshot>
      kinematics_by_key_{}; /**< 逐目标运动学估计缓存（事件填充） */
  boost::signals2::scoped_connection fusion_connection_{}; /**< 融合信号订阅（首次 Step 惰性连接） */
  std::vector<target_inference::TargetInferenceResult> results_{};
};

}  // namespace component_attachment

#endif  // EXAMPLES_COMPONENT_ATTACHMENT_COMPONENTS_INFERENCE_COMPONENT_H_
