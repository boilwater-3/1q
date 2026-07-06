/**
 * @file SbirsOutputTypes.h
 * @brief 定义 SBIRS-inspired 原生观测输出与归属类型。
 */

#ifndef ONEQ_SBIRS_SENSOR_SESSION_SBIRS_OUTPUT_TYPES_H_
#define ONEQ_SBIRS_SENSOR_SESSION_SBIRS_OUTPUT_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/api.hpp"

namespace sbirs_sensor {
namespace output {

enum class ONEQ_API SbirsObservationStage {
  kWideFieldSearch = 0,
  kNarrowFieldAcquisition,
  kNarrowFieldTrack
};

struct ONEQ_API SbirsDetectionRecord {
  std::uint64_t detection_id{0U};
  float azimuth_deg{0.0f};
  float elevation_deg{0.0f};
  float infrared_snr_linear{0.0f};
  SbirsObservationStage observation_stage{SbirsObservationStage::kWideFieldSearch};
  bool detected{false};
};

using SbirsDetectionRecordList = std::vector<SbirsDetectionRecord>;

}  // namespace output

namespace attribution {

struct ONEQ_API SbirsDetectionAttributionRecord {
  std::uint64_t detection_id{0U};
  std::uint64_t target_id{0U};
  std::string target_name{};
  float estimated_range_m{0.0f};
  bool used_truth_assist{false};
};

using SbirsDetectionAttributionRecordList = std::vector<SbirsDetectionAttributionRecord>;

}  // namespace attribution

namespace session {

enum class ONEQ_API SbirsPipelineAbortReason {
  kNone = 0,
  kValidationRejected,
  kOutputContractViolation,
  kRuntimeStateRestoreRejected
};

}  // namespace session
}  // namespace sbirs_sensor

#endif  // ONEQ_SBIRS_SENSOR_SESSION_SBIRS_OUTPUT_TYPES_H_
