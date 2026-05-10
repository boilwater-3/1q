/**
 * @file EosCycleOutputBuilder.h
 * @brief 将内部 EOS 输出帧构建为外部世界坐标输出帧。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_OUTPUT_BUILDER_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_OUTPUT_BUILDER_H_

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
 * @brief 输出侧 builder：把内部 EosOutputFrame 转换为外部 ECEF 输出帧。
 */
struct ONEQ_API EosCycleOutputBuilder {
  static bool Build(const EosExternalPoseInput& platform, const EosOutputFrame& frame,
                    EosExternalOutputFrame* output);

  static bool Build(const EosCoordinateReference& reference,
                    const oneq::foundation::PoseState& platform_pose, const EosOutputFrame& frame,
                    EosExternalOutputFrame* output);

 private:
  EosCycleOutputBuilder() = delete;
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_CYCLE_OUTPUT_BUILDER_H_
