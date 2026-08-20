/**
 * @file EosExternalOutputAdapter.h
 * @brief EOS 外部输出适配统一入口。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_EXTERNAL_OUTPUT_ADAPTER_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_EXTERNAL_OUTPUT_ADAPTER_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"
#include "1q/coordinate/types.h"
#include "1q/electro_optical_sensor/session/EosExternalInputAdapter.h"
#include "1q/electro_optical_sensor/session/EosOutputTypes.h"

namespace electro_optical_sensor {
namespace session {

/**
 * @brief 外部可消费的 EOS 探测输出。
 * @note 内部 EosDetectionRecord 使用相对平台的 range/az/el；本结构补充反解后的 ECEF 位置。
 */
struct ONEQ_API EosExternalDetectionRecord {
  std::uint64_t detection_id{0U}; /**< 探测记录标识 */

  oneq::coordinate::EcefPositionM target_position_ecef_m{}; /**< 目标 ECEF 位置（单位：m） */

  float range_m{0.0f};             /**< 斜距（单位：m） */
  float azimuth_deg{0.0f};         /**< 方位角（单位：deg） */
  float elevation_deg{0.0f};       /**< 仰角（单位：deg） */
  float infrared_snr_linear{0.0f}; /**< 红外通道线性 SNR */
  float visible_snr_linear{0.0f};  /**< 可见光通道线性 SNR */
  float fused_snr_linear{0.0f};    /**< 融合线性 SNR */
  float fused_snr_db{0.0f};        /**< 融合 dB SNR */
  bool detected{false};            /**< 是否通过探测门限判决 */
};

/** @brief EosExternalDetectionRecordList 表示外部 EOS 探测输出集合。 */
using EosExternalDetectionRecordList = std::vector<EosExternalDetectionRecord>;

/**
 * @brief 将内部 EOS 探测记录转换为外部 ECEF 探测记录。
 *
 * 依据平台位姿与局部参考系，将相对平台的 range/az/el 反解为 ECEF 位置，
 * 并原样复制信噪比等标量字段。
 *
 * @param[in] detection 内部探测记录（相对平台的几何量）。
 * @param[in] reference 锚点参考系（`origin_lla` 为平台锚点，决定 ENU→ECEF 基准）。
 * @param[in] platform_attitude_deg 平台姿态角（Body->ENU，单位：deg）。
 * @param[out] output 输出外部 ECEF 探测记录；为 nullptr 时直接返回 false。
 * @return 转换成功返回 true；`output` 为空、几何量非有限/非法、或 ENU→ECEF 变换失败返回 false。
 */
ONEQ_API bool TryMakeExternalDetectionFromRecord(
    const output::EosDetectionRecord& detection,
    const oneq::coordinate::LocalFrameReference& reference,
    const oneq::coordinate::EulerAnglesDeg& platform_attitude_deg,
    EosExternalDetectionRecord* output);

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_EXTERNAL_OUTPUT_ADAPTER_H_
