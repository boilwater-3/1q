/**
 * @file InferenceResult.h
 * @brief 定义目标推演结果（轨迹预测 + 发射/落点 + 类型评估，全部携带误差预算）。
 */

#ifndef ONEQ_TARGET_INFERENCE_INFERENCE_RESULT_H_
#define ONEQ_TARGET_INFERENCE_INFERENCE_RESULT_H_

#include <array>
#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/target_inference/InferenceTrackState.h"

namespace target_inference {

/**
 * @brief 轨迹预测航迹点。
 */
struct ONEQ_API InferenceWaypoint {
  double time_offset_sec{0.0};       /**< 相对输入时刻的时间偏移（未来为正，单位：s）。 */
  oneq::coordinate::LlaPositionDegM position{}; /**< 航迹点位置（度制 LLA）。 */
};

/**
 * @brief 轨迹推演产品（预测/落点/发射点，均携带误差预算）。
 * @note 误差预算为线性化敏度传播（J·P·Jᵀ）；不携带协方差的输入 has_uncertainty=false，
 *       消费方不得把 sigma=0 解释为"零误差"（分层契约规则 6）。
 */
struct ONEQ_API TrajectoryPrediction {
  bool valid{false};                /**< 输入状态是否可推演（有限且在地表上方）。 */
  bool has_uncertainty{false};      /**< 是否携带误差预算（输入含协方差时为真）。 */
  std::vector<InferenceWaypoint> waypoints{}; /**< 前向预测航迹点（含落点前最后点）。 */
  bool has_impact{false};           /**< 预测时域内是否解算出落点。 */
  oneq::coordinate::LlaPositionDegM impact_point{}; /**< 落点（度制 LLA）。 */
  double impact_time_offset_sec{0.0};  /**< 落点时间偏移（单位：s）。 */
  double impact_position_sigma_m{0.0}; /**< 落点 1-σ（单位：m）；误差椭圆参数（半轴/取向）不输出——2026-08-20 验收输出统计裁定不新增（docs/review/acceptance_output_inventory_2026-08-20.md §4.2/§6）。 */
  bool has_launch{false};           /**< 是否解算出发射点。 */
  oneq::coordinate::LlaPositionDegM launch_point{}; /**< 发射点（度制 LLA）。 */
  double launch_time_offset_sec{0.0};  /**< 发射点时间偏移（负值=过去，单位：s）。 */
  double launch_position_sigma_m{0.0}; /**< 发射点 1-σ（单位：m）。 */
  std::array<double, 36U> launch_covariance_ecef{}; /**< 发射点 6×6 协方差（行主序 [x,vx,y,vy,z,vz]）。 */
};

/**
 * @brief 类型评估产品。
 */
struct ONEQ_API TypeAssessment {
  InferenceTargetCategory category{InferenceTargetCategory::kUnknown}; /**< 最可能大类。 */
  double probability{0.0};          /**< 归一化概率 [0,1]。 */
};

/**
 * @brief 单航迹推演结果（与输入顺序一致）。
 */
struct ONEQ_API TargetInferenceResult {
  std::uint64_t key{0U};            /**< 航迹库内键（透传）。 */
  TrajectoryPrediction trajectory{}; /**< 轨迹推演。 */
  TypeAssessment type{};            /**< 类型评估。 */
};

}  // namespace target_inference

#endif  // ONEQ_TARGET_INFERENCE_INFERENCE_RESULT_H_
