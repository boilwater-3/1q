/**
 * @file FusionAcceptanceRecords.h
 * @brief 融合层验收行拼装。
 */

#ifndef ONEQ_SRC_FUSION_FUSION_ACCEPTANCE_RECORDS_H_
#define ONEQ_SRC_FUSION_FUSION_ACCEPTANCE_RECORDS_H_

#include <cstdint>
#include <vector>

#include "1q/fusion/FusedTarget.h"

namespace fusion {

void WriteFusionAcceptance(std::uint32_t cycle, const std::vector<FusedTarget>& tracks,
                           bool filtering_enabled);

}  // namespace fusion

#endif
