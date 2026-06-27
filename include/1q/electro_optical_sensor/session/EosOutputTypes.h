/**
 * @file EosOutputTypes.h
 * @brief EOS 公共输出记录、归属记录与周期终止原因类型。
 */

#ifndef ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_OUTPUT_TYPES_H_
#define ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_OUTPUT_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"

namespace electro_optical_sensor {
namespace output {

/**
 * @brief EosDetectionRecord 表示单条 EOS 传感器探测输出。
 */
struct ONEQ_API EosDetectionRecord {
  std::uint64_t detection_id{0U};  /**< 本输出帧内的探测记录标识 */
  float range_m{0.0f};             /**< 斜距（单位：m） */
  float azimuth_deg{0.0f};         /**< 方位角（单位：deg） */
  float elevation_deg{0.0f};       /**< 仰角（单位：deg） */
  float infrared_snr_linear{0.0f}; /**< 红外通道线性 SNR */
  float visible_snr_linear{0.0f};  /**< 可见光通道线性 SNR */
  float fused_snr_linear{0.0f};    /**< 融合线性 SNR */
  float fused_snr_db{0.0f};        /**< 融合 dB SNR */
  bool detected{false};            /**< 是否通过探测门限判决 */
};

/** @brief EosDetectionRecordList 表示单周期探测结果列表。 */
using EosDetectionRecordList = std::vector<EosDetectionRecord>;

}  // namespace output

namespace attribution {

/**
 * @brief EosDetectionAttributionRecord 表示仿真归属映射。
 * @note 该类型不属于真实传感器输出；只用于 StepWithResult、调试视图、生命周期和 replay 诊断。
 */
struct ONEQ_API EosDetectionAttributionRecord {
  std::uint64_t detection_id{0U}; /**< 对应 EosDetectionRecord::detection_id */
  std::uint64_t target_id{0U};    /**< 仿真输入目标 ID */
  std::string target_name{};      /**< 仿真输入目标名称 */
};

/** @brief EosDetectionAttributionRecordList 表示探测记录到仿真目标的归属映射集合。 */
using EosDetectionAttributionRecordList = std::vector<EosDetectionAttributionRecord>;

}  // namespace attribution

namespace session {

/**
 * @brief EosPipelineAbortReason 描述核心管线周期终止原因。
 */
enum class ONEQ_API EosPipelineAbortReason {
  kNone = 0,
  kValidationRejected,
  kOutputContractViolation,
  kRuntimeStateRestoreRejected
};

}  // namespace session
}  // namespace electro_optical_sensor

#endif  // ONEQ_ELECTRO_OPTICAL_SENSOR_SESSION_EOS_OUTPUT_TYPES_H_
