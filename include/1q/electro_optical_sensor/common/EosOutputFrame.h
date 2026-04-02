/**
 * @file EosOutputFrame.h
 * @brief 定义光学传感器单周期探测输出结构。
 */

#ifndef ELECTRO_OPTICAL_SENSOR_COMMON_EOS_OUTPUT_FRAME_H_
#define ELECTRO_OPTICAL_SENSOR_COMMON_EOS_OUTPUT_FRAME_H_

#include <cstdint>
#include <vector>

#include "1q/api.hpp"

namespace electro_optical_sensor {
namespace common {

/**
 * @brief EosDetectionRecord 表示单目标探测输出条目。
 */
struct ONEQ_API EosDetectionRecord {
  std::uint64_t target_id{0U};      /**< 目标标识 */
  float range_m{0.0f};              /**< 斜距（单位：m） */
  float azimuth_deg{0.0f};          /**< 方位角（单位：deg） */
  float elevation_deg{0.0f};        /**< 仰角（单位：deg） */
  float infrared_snr_linear{0.0f};  /**< 红外通道线性 SNR */
  float visible_snr_linear{0.0f};   /**< 可见光通道线性 SNR */
  float fused_snr_linear{0.0f};     /**< 融合线性 SNR */
  float fused_snr_db{0.0f};         /**< 融合 dB SNR */
  bool detected{false};             /**< 是否通过探测门限判决 */
};

/** @brief EosDetectionRecordList 表示单周期探测结果列表。 */
using EosDetectionRecordList = std::vector<EosDetectionRecord>;

/**
 * @brief EosOutputFrame 表示单周期聚合探测输出帧。
 */
struct ONEQ_API EosOutputFrame {
  std::uint32_t cycle_index{0U};           /**< 当前周期号 */
  float scan_azimuth_deg{0.0f};            /**< 当前周期扫描中心方位角（单位：deg） */
  EosDetectionRecordList detections{};     /**< 当前周期探测结果 */
};

}  // namespace common
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_COMMON_EOS_OUTPUT_FRAME_H_
