/**
 * @file EosCycleOutputAdapter.h
 * @brief 将内部 EOS 输出帧构建为外部世界坐标输出帧。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_OUTPUT_ADAPTER_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_OUTPUT_ADAPTER_H_

#include "1q/api.hpp"
#include "1q/electro_optical_sensor/session/EosCycleResult.h"
#include "1q/electro_optical_sensor/session/EosExternalOutputAdapter.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief 外部可消费的 EOS 单周期输出帧。
 */
struct ONEQ_API EosExternalOutputFrame {
  std::uint32_t cycle_index{0U};               /**< 当前周期号 */
  float scan_azimuth_deg{0.0f};                /**< 当前周期扫描中心方位角（单位：deg） */
  EosExternalDetectionRecordList detections{}; /**< 外部 ECEF 探测输出列表 */
};

/**
 * @brief 输出侧适配器：把内部 EosOutputFrame 转换为外部 ECEF 输出帧。
 */
struct ONEQ_API EosCycleOutputAdapter {
  /**
   * @brief 由外部平台位姿推导局部参考系，再将内部输出帧转换为外部 ECEF 输出帧。
   * @param[in] platform 外部平台运动学输入（ECEF 位置 + 姿态）。
   * @param[in] frame 本周期内部输出帧。
   * @param[out] output 输出外部 ECEF 输出帧；为 nullptr 时直接返回 false。
   * @return 转换成功返回 true；`output` 为空或任一坐标变换失败返回 false。
   */
  static bool Build(const EosExternalPoseInput& platform, const EosOutputFrame& frame,
                    EosExternalOutputFrame* output);

  /**
   * @brief 使用显式局部参考系，将内部输出帧转换为外部 ECEF 输出帧。
   * @param[in] reference 局部参考系（原点 LLA + 姿态）。
   * @param[in] platform_pose 平台局部位姿状态。
   * @param[in] frame 本周期内部输出帧。
   * @param[out] output 输出外部 ECEF 输出帧；为 nullptr 时直接返回 false。
   * @return 转换成功返回 true；`output` 为空或任一探测记录坐标变换失败返回 false。
   */
  static bool Build(const oneq::coordinate::LocalFrameReference& reference,
                    const oneq::foundation::PoseState& platform_pose, const EosOutputFrame& frame,
                    EosExternalOutputFrame* output);

 private:
  EosCycleOutputAdapter() = delete;
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_OUTPUT_ADAPTER_H_
