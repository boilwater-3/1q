/**
 * @file ObservationPreprocessor.h
 * @brief 定义 ESR 观测预处理器。
 */

#ifndef ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_OBSERVATION_PREPROCESSOR_H_
#define ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_OBSERVATION_PREPROCESSOR_H_

#include <vector>

#include "1q/electronic_surveillance_radar/pipeline/InterceptPipelineTypes.h"
#include "electronic_surveillance_radar/pipeline/ObservationPipelineTypes.h"

namespace electronic_surveillance_radar {
namespace pipeline {
namespace internal {

/**
 * @brief ObservationPreprocessor 负责观测排序、过滤和去重。
 */
class ObservationPreprocessor final {
 public:
  /**
   * @brief 对输入观测执行预处理。
   * @param[in] records 输入观测记录列表。
   * @param[in] config 预处理配置。
   * @return 预处理后的观测记录列表。
   */
  std::vector<RawObservationRecord> Run(const std::vector<RawObservationRecord>& records,
                                        const InterceptPreprocessConfig& config) const;
};

}  // namespace internal
}  // namespace pipeline
}  // namespace electronic_surveillance_radar

#endif  // ELECTRONIC_SURVEILLANCE_RADAR_SRC_PIPELINE_OBSERVATION_PREPROCESSOR_H_
