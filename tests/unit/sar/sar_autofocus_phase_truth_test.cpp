#include <gtest/gtest.h>

#include <limits>

#include "sar/imaging/SarAutofocusPhaseTruth.h"

namespace sar {
namespace imaging {
namespace {

TEST(SarAutofocusPhaseTruthTest, BuildsNormalizedApertureCoordinates) {
  AutofocusPhaseTruthConfig config;
  config.sample_count = 5U;
  AutofocusPhaseTruthDiagnostics diagnostics;
  ASSERT_TRUE(EvaluateAutofocusPhaseTruth(config, &diagnostics));
  ASSERT_EQ(diagnostics.normalized_aperture_coordinates.size(), 5U);
  EXPECT_DOUBLE_EQ(diagnostics.normalized_aperture_coordinates.front(), -1.0);
  EXPECT_DOUBLE_EQ(diagnostics.normalized_aperture_coordinates[2], 0.0);
  EXPECT_DOUBLE_EQ(diagnostics.normalized_aperture_coordinates.back(), 1.0);
}

TEST(SarAutofocusPhaseTruthTest, RemovesConstantAndLinearUnobservableComponents) {
  AutofocusPhaseTruthConfig config;
  config.sample_count = 7U;
  config.constant_rad = 3.0;
  config.linear_rad = -2.0;
  AutofocusPhaseTruthDiagnostics diagnostics;
  ASSERT_TRUE(EvaluateAutofocusPhaseTruth(config, &diagnostics));
  EXPECT_NEAR(diagnostics.fitted_unobservable_constant_rad, 3.0, 1.0e-12);
  EXPECT_NEAR(diagnostics.fitted_unobservable_linear_rad, -2.0, 1.0e-12);
  EXPECT_NEAR(diagnostics.observable_max_abs_rad, 0.0, 1.0e-12);
}

TEST(SarAutofocusPhaseTruthTest, PreservesObservableTruthAndBuildsOppositeCorrection) {
  AutofocusPhaseTruthConfig config;
  config.sample_count = 9U;
  config.constant_rad = 1.0;
  config.linear_rad = 2.0;
  config.quadratic_rad = 0.7;
  config.cubic_rad = -0.4;
  AutofocusPhaseTruthDiagnostics diagnostics;
  ASSERT_TRUE(EvaluateAutofocusPhaseTruth(config, &diagnostics));
  EXPECT_GT(diagnostics.observable_rms_rad, 0.0);
  EXPECT_NEAR(diagnostics.removal_residual_mean_rad, 0.0, 1.0e-12);
  EXPECT_NEAR(diagnostics.removal_residual_linear_projection_rad, 0.0, 1.0e-12);
  for (std::size_t index = 0U; index < config.sample_count; ++index) {
    EXPECT_NEAR(diagnostics.raw_phase_error_rad[index],
                diagnostics.unobservable_phase_rad[index] +
                    diagnostics.observable_phase_error_rad[index],
                1.0e-12);
    EXPECT_DOUBLE_EQ(diagnostics.correction_phase_rad[index],
                     -diagnostics.observable_phase_error_rad[index]);
  }
}

TEST(SarAutofocusPhaseTruthTest, RejectsInvalidInputAndIsDeterministic) {
  AutofocusPhaseTruthConfig config;
  config.sample_count = 5U;
  config.quadratic_rad = 0.5;
  AutofocusPhaseTruthDiagnostics first;
  AutofocusPhaseTruthDiagnostics second;
  ASSERT_TRUE(EvaluateAutofocusPhaseTruth(config, &first));
  ASSERT_TRUE(EvaluateAutofocusPhaseTruth(config, &second));
  EXPECT_EQ(first.raw_phase_error_rad, second.raw_phase_error_rad);
  EXPECT_EQ(first.observable_phase_error_rad, second.observable_phase_error_rad);

  config.sample_count = 2U;
  EXPECT_FALSE(EvaluateAutofocusPhaseTruth(config, &first));
  EXPECT_FALSE(first.valid);
  config.sample_count = 5U;
  config.cubic_rad = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(EvaluateAutofocusPhaseTruth(config, &first));
  EXPECT_FALSE(EvaluateAutofocusPhaseTruth(config, nullptr));
}

}  // namespace
}  // namespace imaging
}  // namespace sar
