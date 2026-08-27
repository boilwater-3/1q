/**
 * @file PrecisionAcceptanceRecords.h
 * @brief 精度评估验收行拼装。
 */

#ifndef ONEQ_SRC_PRECISION_EVALUATION_PRECISION_ACCEPTANCE_RECORDS_H_
#define ONEQ_SRC_PRECISION_EVALUATION_PRECISION_ACCEPTANCE_RECORDS_H_

#include <vector>

#include "1q/precision_evaluation/PrecisionEvaluationTypes.h"

namespace precision_evaluation {

void WritePrecisionKeyMetrics(const PrecisionEvaluationReport& report,
                              const std::vector<double>& east_m, const std::vector<double>& north_m,
                              const std::vector<double>& up_m, const std::vector<double>& az_deg,
                              const std::vector<double>& el_deg,
                              const std::vector<double>& slant_range_m);

void WritePrecisionAhp(const PrecisionEvaluationReport& report);

}  // namespace precision_evaluation

#endif
