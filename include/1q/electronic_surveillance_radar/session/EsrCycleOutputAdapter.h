/**
 * @file EsrCycleOutputAdapter.h
 * @brief 将内部 ESR 输出帧构建为外部世界坐标输出帧。
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_OUTPUT_ADAPTER_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_OUTPUT_ADAPTER_H_

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
  session::TruthEvaluationFrame truth_evaluation_output{}; /**< 真值评估输出通道 */
};

/**
 * @brief 输出侧适配器：把内部 EsrOutputFrame 转换为外部 ECEF 方位线输出帧。
 */
struct ONEQ_API EsrCycleOutputAdapter {
  /**
   * @brief 从外部平台运动学输入一步构建外部输出帧。
   *
   * 内部先由 ECEF 位置反解 LLA 原点并构造局部参考系，再委托四参重载完成坐标转换。
   *
   * @param[in] platform 外部平台运动学输入。
   * @param[in] frame 内部三通道输出帧。
   * @param[out] output 输出外部输出帧。
   * @return 全部转换成功返回 true；`output` 为 nullptr 或坐标转换失败返回 false。
   */
  static bool Build(const EsrExternalPoseInput& platform, const EsrOutputFrame& frame,
                    EsrExternalOutputFrame* output);

  /**
   * @brief 用显式局部参考系与平台姿态构建外部输出帧。
   *
   * 将观测与假设的局部方位线逐一转换为 ECEF 单位方位线，并直通真值评估通道。
   *
   * @param[in] reference ESR 局部坐标参考系（ENU 原点 + frame 姿态）。
   * @param[in] platform_pose 侦察平台姿态状态。
   * @param[in] frame 内部三通道输出帧。
   * @param[out] output 输出外部输出帧。
   * @return 全部转换成功返回 true；`output` 为 nullptr 或任一记录转换失败返回 false。
   */
  static bool Build(const oneq::coordinate::LocalFrameReference& reference, const oneq::foundation::PoseState& platform_pose,
                    const EsrOutputFrame& frame, EsrExternalOutputFrame* output);

 private:
  EsrCycleOutputAdapter() = delete;
};

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_CYCLE_OUTPUT_ADAPTER_H_
