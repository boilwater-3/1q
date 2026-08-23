/**
 * @file InferenceAcceptanceRecords.h
 * @brief 推演层验收行拼装。
 */

#ifndef ONEQ_SRC_TARGET_INFERENCE_INFERENCE_ACCEPTANCE_RECORDS_H_
#define ONEQ_SRC_TARGET_INFERENCE_INFERENCE_ACCEPTANCE_RECORDS_H_

#include <array>
#include <cstdint>
#include <vector>

#include "1q/coordinate/types.h"
#include "1q/target_inference/InferenceResult.h"
#include "1q/target_inference/InferenceTrackState.h"
#include "1q/target_inference/TargetInferenceConfig.h"

namespace target_inference {

/**
 * @brief 比机械能（单位：J/kg）：ε = ½|v|² − μ/|r|。
 * @param[in] position       ECEF 位置（m）。
 * @param[in] velocity_ecef  ECEF 速度（m/s）。
 * @param[in] earth_mu_m3_per_s2 地球引力参数（m³/s²，引擎配置单源传入）。
 * @return 位置范数 > 0 时返回比机械能；位置为零矢时返回 0（不可用，调用方跳过）。
 * @note 不受验收日志宏门控，恒编译可测；μ 不在本文件新造常数。
 */
double SpecificMechanicalEnergyJPerKg(const oneq::coordinate::EcefPositionM& position,
                                      const std::array<double, 3U>& velocity_ecef_m_per_s,
                                      double earth_mu_m3_per_s2);

/**
 * @brief 推演验收行写入。μ 与地球半径取引擎配置（TargetInferenceConfig 单源）。
 */
void WriteInferenceAcceptance(const std::vector<InferenceTrackState>& tracks,
                              const std::vector<TargetInferenceResult>& results,
                              const TargetInferenceConfig& config);

}  // namespace target_inference

#endif
