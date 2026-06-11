/**
 * @file SarCsaIntermediateTruth.h
 * @brief CSA 显式操作列表与逐阶段中间域真值执行器。
 */

#ifndef ONEQ_SRC_SAR_IMAGING_SAR_CSA_INTERMEDIATE_TRUTH_H_
#define ONEQ_SRC_SAR_IMAGING_SAR_CSA_INTERMEDIATE_TRUTH_H_

#include <cstddef>
#include <string>
#include <vector>

#include "sar/signal/SarFft.h"

namespace sar {
namespace imaging {

enum class CsaTruthOperation {
  kForwardRangeFft = 0,
  kInverseRangeFft = 1,
  kForwardAzimuthFft = 2,
  kInverseAzimuthFft = 3,
  kMultiplyPhaseKernel = 4,
};

enum class CsaTruthExecutionStatus {
  kSucceeded = 0,
  kRejected = 1,
};

enum class CsaTruthRejectionReason {
  kNone = 0,
  kInvalidReferenceId = 1,
  kInvalidInput = 2,
  kEmptyStages = 3,
  kInvalidStage = 4,
  kInvalidPhaseKernel = 5,
  kOperationFailure = 6,
  kTruthMismatch = 7,
};

struct CsaTruthStage {
  std::string stage_id;
  CsaTruthOperation operation{CsaTruthOperation::kForwardRangeFft};
  std::string phase_kernel_id;
  signal::ComplexMatrix phase_kernel;
  signal::ComplexMatrix expected_output;
  double maximum_abs_error_tolerance{0.0};
  double unit_energy_nrms_tolerance{0.0};
};

struct CsaIntermediateTruthReference {
  std::string reference_id;
  signal::ComplexMatrix input;
  std::vector<CsaTruthStage> stages;
};

struct CsaTruthStageDiagnostics {
  std::string stage_id;
  CsaTruthOperation operation{CsaTruthOperation::kForwardRangeFft};
  double maximum_abs_error{0.0};
  double unit_energy_nrms{0.0};
  bool passed{false};
};

struct CsaIntermediateTruthResult {
  CsaTruthExecutionStatus status{CsaTruthExecutionStatus::kRejected};
  CsaTruthRejectionReason reason{CsaTruthRejectionReason::kNone};
  std::size_t first_failed_stage_index{static_cast<std::size_t>(-1)};
  std::vector<CsaTruthStageDiagnostics> stage_diagnostics;
  signal::ComplexMatrix final_output;
};

CsaIntermediateTruthResult ExecuteCsaIntermediateTruthReference(
    const CsaIntermediateTruthReference& reference);

}  // namespace imaging
}  // namespace sar

#endif  // ONEQ_SRC_SAR_IMAGING_SAR_CSA_INTERMEDIATE_TRUTH_H_
