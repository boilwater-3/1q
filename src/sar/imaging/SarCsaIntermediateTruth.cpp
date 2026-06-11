#include "sar/imaging/SarCsaIntermediateTruth.h"

#include <algorithm>
#include <cmath>

namespace sar {
namespace imaging {

namespace {

constexpr double kUnitMagnitudeTolerance = 1.0e-12;

bool HasValidShape(const signal::ComplexMatrix& matrix) {
  return matrix.rows > 0U && matrix.cols > 0U &&
         matrix.values.size() == matrix.rows * matrix.cols;
}

bool HasSameShape(const signal::ComplexMatrix& first, const signal::ComplexMatrix& second) {
  return first.rows == second.rows && first.cols == second.cols &&
         first.values.size() == second.values.size();
}

bool IsFiniteMatrix(const signal::ComplexMatrix& matrix) {
  if (!HasValidShape(matrix)) {
    return false;
  }
  for (const signal::ComplexSample& sample : matrix.values) {
    if (!std::isfinite(sample.real()) || !std::isfinite(sample.imag())) {
      return false;
    }
  }
  return true;
}

bool IsUnitMagnitudeKernel(const signal::ComplexMatrix& kernel) {
  if (!IsFiniteMatrix(kernel)) {
    return false;
  }
  for (const signal::ComplexSample& sample : kernel.values) {
    if (std::abs(std::abs(sample) - 1.0) > kUnitMagnitudeTolerance) {
      return false;
    }
  }
  return true;
}

bool IsValidTolerance(double tolerance) {
  return std::isfinite(tolerance) && tolerance >= 0.0;
}

bool ApplyStage(const CsaTruthStage& stage, const signal::ComplexMatrix& input,
                signal::ComplexMatrix* output) {
  switch (stage.operation) {
    case CsaTruthOperation::kForwardRangeFft:
      return signal::FftRows(input, false, output);
    case CsaTruthOperation::kInverseRangeFft:
      return signal::FftRows(input, true, output);
    case CsaTruthOperation::kForwardAzimuthFft:
      return signal::FftCols(input, false, output);
    case CsaTruthOperation::kInverseAzimuthFft:
      return signal::FftCols(input, true, output);
    case CsaTruthOperation::kMultiplyPhaseKernel:
      if (stage.phase_kernel_id.empty() || !IsUnitMagnitudeKernel(stage.phase_kernel) ||
          !HasSameShape(input, stage.phase_kernel)) {
        return false;
      }
      *output = input;
      for (std::size_t index = 0U; index < output->values.size(); ++index) {
        output->values[index] *= stage.phase_kernel.values[index];
      }
      return true;
  }
  return false;
}

CsaTruthStageDiagnostics CompareStage(const CsaTruthStage& stage,
                                      const signal::ComplexMatrix& actual) {
  CsaTruthStageDiagnostics diagnostics;
  diagnostics.stage_id = stage.stage_id;
  diagnostics.operation = stage.operation;
  double expected_energy = 0.0;
  double actual_energy = 0.0;
  double error_energy = 0.0;
  for (std::size_t index = 0U; index < actual.values.size(); ++index) {
    diagnostics.maximum_abs_error =
        std::max(diagnostics.maximum_abs_error,
                 std::abs(actual.values[index] - stage.expected_output.values[index]));
    expected_energy += std::norm(stage.expected_output.values[index]);
    actual_energy += std::norm(actual.values[index]);
  }
  if (expected_energy > 0.0 && actual_energy > 0.0) {
    const double expected_scale = 1.0 / std::sqrt(expected_energy);
    const double actual_scale = 1.0 / std::sqrt(actual_energy);
    for (std::size_t index = 0U; index < actual.values.size(); ++index) {
      error_energy +=
          std::norm(actual.values[index] * actual_scale -
                    stage.expected_output.values[index] * expected_scale);
    }
    diagnostics.unit_energy_nrms = std::sqrt(error_energy);
  } else {
    diagnostics.unit_energy_nrms = diagnostics.maximum_abs_error;
  }
  diagnostics.passed =
      diagnostics.maximum_abs_error <= stage.maximum_abs_error_tolerance &&
      diagnostics.unit_energy_nrms <= stage.unit_energy_nrms_tolerance;
  return diagnostics;
}

CsaIntermediateTruthResult Reject(CsaTruthRejectionReason reason,
                                  std::size_t stage_index,
                                  const std::vector<CsaTruthStageDiagnostics>& diagnostics) {
  CsaIntermediateTruthResult result;
  result.reason = reason;
  result.first_failed_stage_index = stage_index;
  result.stage_diagnostics = diagnostics;
  return result;
}

}  // namespace

CsaIntermediateTruthResult ExecuteCsaIntermediateTruthReference(
    const CsaIntermediateTruthReference& reference) {
  if (reference.reference_id.empty()) {
    return Reject(CsaTruthRejectionReason::kInvalidReferenceId,
                  static_cast<std::size_t>(-1), {});
  }
  if (!IsFiniteMatrix(reference.input)) {
    return Reject(CsaTruthRejectionReason::kInvalidInput, static_cast<std::size_t>(-1), {});
  }
  if (reference.stages.empty()) {
    return Reject(CsaTruthRejectionReason::kEmptyStages, static_cast<std::size_t>(-1), {});
  }

  signal::ComplexMatrix current = reference.input;
  std::vector<CsaTruthStageDiagnostics> diagnostics;
  for (std::size_t index = 0U; index < reference.stages.size(); ++index) {
    const CsaTruthStage& stage = reference.stages[index];
    if (stage.stage_id.empty() || !IsFiniteMatrix(stage.expected_output) ||
        !HasSameShape(current, stage.expected_output) ||
        !IsValidTolerance(stage.maximum_abs_error_tolerance) ||
        !IsValidTolerance(stage.unit_energy_nrms_tolerance)) {
      return Reject(CsaTruthRejectionReason::kInvalidStage, index, diagnostics);
    }
    if (stage.operation == CsaTruthOperation::kMultiplyPhaseKernel &&
        (stage.phase_kernel_id.empty() || !IsUnitMagnitudeKernel(stage.phase_kernel) ||
         !HasSameShape(current, stage.phase_kernel))) {
      return Reject(CsaTruthRejectionReason::kInvalidPhaseKernel, index, diagnostics);
    }

    signal::ComplexMatrix output;
    if (!ApplyStage(stage, current, &output) || !HasSameShape(output, stage.expected_output)) {
      return Reject(CsaTruthRejectionReason::kOperationFailure, index, diagnostics);
    }
    diagnostics.push_back(CompareStage(stage, output));
    if (!diagnostics.back().passed) {
      return Reject(CsaTruthRejectionReason::kTruthMismatch, index, diagnostics);
    }
    current = output;
  }

  CsaIntermediateTruthResult result;
  result.status = CsaTruthExecutionStatus::kSucceeded;
  result.reason = CsaTruthRejectionReason::kNone;
  result.stage_diagnostics = diagnostics;
  result.final_output = current;
  return result;
}

}  // namespace imaging
}  // namespace sar
