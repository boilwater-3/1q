/**
 * @file EsrCycleOutputBuilder.h
 * @brief 将内部 ESR 输出帧构建为外部世界坐标输出帧。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_OUTPUT_BUILDER_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_OUTPUT_BUILDER_H_

#include "1q/api.hpp"
#include "1q/electronic_surveillance_radar/session/EsrCycleResult.h"
#include "1q/electronic_surveillance_radar/session/EsrExternalOutputAdapter.h"

namespace electronic_surveillance_radar {
namespace session {

/**
 * @brief 外部可消费的 ESR 单周期输出帧。
 */
struct ONEQ_API EsrExternalOutputFrame {
  std::uint32_t cycle_index{0U};                             /**< 当前周期号 */
  std::uint64_t batch_id{0U};                                /**< 当前批次号 */
  EsrExternalObservationList observations{};                 /**< 外部观测输出 */
  EsrExternalEmitterHypothesisList hypotheses{};             /**< 外部假设输出 */
  extension::TruthEvaluationFrame truth_evaluation_output{}; /**< 真值评估输出通道 */
};

/**
 * @brief 输出侧 builder：把内部 EsrOutputFrame 转换为外部 ECEF 方位线输出帧。
 */
struct ONEQ_API EsrCycleOutputBuilder {
  static bool Build(const EsrExternalPoseInput& platform, const EsrOutputFrame& frame,
                    EsrExternalOutputFrame* output);

  static bool Build(const oneq::coordinate::LocalFrameReference& reference, const oneq::foundation::PoseState& platform_pose,
                    const EsrOutputFrame& frame, EsrExternalOutputFrame* output);

 private:
  EsrCycleOutputBuilder() = delete;
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_OUTPUT_BUILDER_H_
