#include <gtest/gtest.h>

#include <complex>

#include "sar/imaging/SarCsaIntermediateTruth.h"

namespace sar {
namespace imaging {
namespace {

signal::ComplexMatrix Matrix(std::initializer_list<signal::ComplexSample> values) {
  signal::ComplexMatrix matrix;
  matrix.rows = 2U;
  matrix.cols = 2U;
  matrix.values.assign(values);
  return matrix;
}

CsaTruthStage Stage(const char* id, CsaTruthOperation operation,
                    const signal::ComplexMatrix& expected) {
  CsaTruthStage stage;
  stage.stage_id = id;
  stage.operation = operation;
  stage.expected_output = expected;
  stage.maximum_abs_error_tolerance = 1.0e-12;
  stage.unit_energy_nrms_tolerance = 1.0e-12;
  return stage;
}

CsaIntermediateTruthReference ValidReference() {
  const signal::ComplexSample i(0.0, 1.0);
  CsaIntermediateTruthReference reference;
  reference.reference_id = "synthetic-axis-sign-truth";
  reference.input = Matrix({1.0, 0.0, 0.0, 0.0});
  reference.stages.push_back(
      Stage("range-forward", CsaTruthOperation::kForwardRangeFft,
            Matrix({1.0, 1.0, 0.0, 0.0})));
  CsaTruthStage phase =
      Stage("phase", CsaTruthOperation::kMultiplyPhaseKernel,
            Matrix({1.0, i, 0.0, 0.0}));
  phase.phase_kernel_id = "synthetic-unit-phase";
  phase.phase_kernel = Matrix({1.0, i, -1.0, -i});
  reference.stages.push_back(phase);
  reference.stages.push_back(
      Stage("azimuth-forward", CsaTruthOperation::kForwardAzimuthFft,
            Matrix({1.0, i, 1.0, i})));
  reference.stages.push_back(
      Stage("range-inverse", CsaTruthOperation::kInverseRangeFft,
            Matrix({signal::ComplexSample(0.5, 0.5), signal::ComplexSample(0.5, -0.5),
                    signal::ComplexSample(0.5, 0.5), signal::ComplexSample(0.5, -0.5)})));
  return reference;
}

TEST(SarCsaIntermediateTruthTest, ReplaysExplicitStagesAndReturnsAtomicFinalOutput) {
  const CsaIntermediateTruthReference reference = ValidReference();
  const CsaIntermediateTruthResult result = ExecuteCsaIntermediateTruthReference(reference);

  ASSERT_EQ(result.status, CsaTruthExecutionStatus::kSucceeded);
  EXPECT_EQ(result.reason, CsaTruthRejectionReason::kNone);
  ASSERT_EQ(result.stage_diagnostics.size(), reference.stages.size());
  for (const CsaTruthStageDiagnostics& diagnostics : result.stage_diagnostics) {
    EXPECT_TRUE(diagnostics.passed);
    EXPECT_LE(diagnostics.maximum_abs_error, 1.0e-12);
    EXPECT_LE(diagnostics.unit_energy_nrms, 1.0e-12);
  }
  EXPECT_EQ(result.final_output.values, reference.stages.back().expected_output.values);
}

TEST(SarCsaIntermediateTruthTest, PreservesAsymmetricComplexMatrixAcrossRangeRoundTrip) {
  const signal::ComplexSample i(0.0, 1.0);
  CsaIntermediateTruthReference reference;
  reference.reference_id = "asymmetric-complex-range-round-trip";
  reference.input = Matrix({1.0, i, 2.0, 3.0 * i});
  reference.stages.push_back(
      Stage("range-forward", CsaTruthOperation::kForwardRangeFft,
            Matrix({1.0 + i, 1.0 - i, 2.0 + 3.0 * i, 2.0 - 3.0 * i})));
  reference.stages.push_back(
      Stage("range-inverse", CsaTruthOperation::kInverseRangeFft, reference.input));

  const CsaIntermediateTruthResult result = ExecuteCsaIntermediateTruthReference(reference);

  ASSERT_EQ(result.status, CsaTruthExecutionStatus::kSucceeded);
  EXPECT_EQ(result.final_output.values, reference.input.values);
}

TEST(SarCsaIntermediateTruthTest, RejectsInvalidKernelAndLeavesFinalOutputEmpty) {
  CsaIntermediateTruthReference reference = ValidReference();
  reference.stages[1].phase_kernel.values[0] = 2.0;
  const CsaIntermediateTruthResult result = ExecuteCsaIntermediateTruthReference(reference);

  EXPECT_EQ(result.status, CsaTruthExecutionStatus::kRejected);
  EXPECT_EQ(result.reason, CsaTruthRejectionReason::kInvalidPhaseKernel);
  EXPECT_EQ(result.first_failed_stage_index, 1U);
  EXPECT_TRUE(result.final_output.values.empty());
}

TEST(SarCsaIntermediateTruthTest, RejectsFirstTruthMismatchWithDiagnostics) {
  CsaIntermediateTruthReference reference = ValidReference();
  reference.stages[2].expected_output.values[0] += 0.1;
  const CsaIntermediateTruthResult result = ExecuteCsaIntermediateTruthReference(reference);

  EXPECT_EQ(result.status, CsaTruthExecutionStatus::kRejected);
  EXPECT_EQ(result.reason, CsaTruthRejectionReason::kTruthMismatch);
  EXPECT_EQ(result.first_failed_stage_index, 2U);
  ASSERT_EQ(result.stage_diagnostics.size(), 3U);
  EXPECT_FALSE(result.stage_diagnostics.back().passed);
  EXPECT_TRUE(result.final_output.values.empty());
}

TEST(SarCsaIntermediateTruthTest, IsDeterministicAndDoesNotModifyReference) {
  const CsaIntermediateTruthReference reference = ValidReference();
  const CsaIntermediateTruthReference before = reference;
  const CsaIntermediateTruthResult first = ExecuteCsaIntermediateTruthReference(reference);
  const CsaIntermediateTruthResult second = ExecuteCsaIntermediateTruthReference(reference);

  EXPECT_EQ(first.final_output.values, second.final_output.values);
  EXPECT_EQ(reference.input.values, before.input.values);
  ASSERT_EQ(reference.stages.size(), before.stages.size());
  for (std::size_t index = 0U; index < reference.stages.size(); ++index) {
    EXPECT_EQ(reference.stages[index].expected_output.values,
              before.stages[index].expected_output.values);
    EXPECT_EQ(reference.stages[index].phase_kernel.values,
              before.stages[index].phase_kernel.values);
  }
}

}  // namespace
}  // namespace imaging
}  // namespace sar
