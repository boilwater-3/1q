/**
 * @file InferenceAcceptanceRecords.h
 * @brief 推演层验收行拼装。
 */

#ifndef ONEQ_SRC_TARGET_INFERENCE_INFERENCE_ACCEPTANCE_RECORDS_H_
#define ONEQ_SRC_TARGET_INFERENCE_INFERENCE_ACCEPTANCE_RECORDS_H_

#include <cstdint>
#include <vector>

#include "1q/target_inference/InferenceResult.h"
#include "1q/target_inference/InferenceTrackState.h"

namespace target_inference {

void WriteInferenceAcceptance(const std::vector<InferenceTrackState>& tracks,
                              const std::vector<TargetInferenceResult>& results);

}  // namespace target_inference

#endif
