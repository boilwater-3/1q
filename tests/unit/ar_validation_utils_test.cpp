/**
 * @file validation_utils_test.cpp
 * @brief 验证校验通用工具的有限值、问题构造与严重级别扫描行为。
 */

#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "1q/airborne_radar/session/RadarInputValidation.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"
#include "common/validation/ValidationUtils.h"

namespace oneq {
namespace internal {
namespace validation {
namespace {

TEST(ValidationUtilsTest, IsFiniteDistinguishesFiniteNanAndInfinity) {
  EXPECT_TRUE(IsFinite(1.0f));
  EXPECT_TRUE(IsFinite(3.5));
  EXPECT_FALSE(IsFinite(std::numeric_limits<float>::quiet_NaN()));
  EXPECT_FALSE(IsFinite(std::numeric_limits<double>::infinity()));
}

TEST(ValidationUtilsTest, MakeIndexedIssueSupportsTargetIndexField) {
  const airborne_radar::session::ValidationIssue issue =
      MakeIndexedIssue<airborne_radar::session::ValidationIssue,
                       airborne_radar::session::ValidationSeverity,
                       airborne_radar::session::ValidationCode,
                       &airborne_radar::session::ValidationIssue::target_index>(
          airborne_radar::session::ValidationSeverity::kWarning,
          airborne_radar::session::ValidationCode::kNegativeRcs, 3U, "negative rcs");

  EXPECT_EQ(issue.severity, airborne_radar::session::ValidationSeverity::kWarning);
  EXPECT_EQ(issue.code, airborne_radar::session::ValidationCode::kNegativeRcs);
  EXPECT_EQ(issue.target_index, 3U);
  EXPECT_EQ(issue.message, "negative rcs");
}

TEST(ValidationUtilsTest, MakeIndexedIssueSupportsEmitterIndexField) {
  const electronic_surveillance_radar::session::EsrValidationIssue issue = MakeIndexedIssue<
      electronic_surveillance_radar::session::EsrValidationIssue,
      electronic_surveillance_radar::session::EsrValidationSeverity,
      electronic_surveillance_radar::session::EsrValidationCode,
      &electronic_surveillance_radar::session::EsrValidationIssue::emitter_index>(
      electronic_surveillance_radar::session::EsrValidationSeverity::kError,
      electronic_surveillance_radar::session::EsrValidationCode::kInvalidEmitterFrequency, 5U,
      "invalid emitter frequency");

  EXPECT_EQ(issue.severity,
            electronic_surveillance_radar::session::EsrValidationSeverity::kError);
  EXPECT_EQ(
      issue.code,
      electronic_surveillance_radar::session::EsrValidationCode::kInvalidEmitterFrequency);
  EXPECT_EQ(issue.emitter_index, 5U);
  EXPECT_EQ(issue.message, "invalid emitter frequency");
}

TEST(ValidationUtilsTest, HasSeverityReturnsFalseForEmptyList) {
  const std::vector<airborne_radar::session::ValidationIssue> issues;
  const bool has_error = HasSeverity<std::vector<airborne_radar::session::ValidationIssue>,
                                     airborne_radar::session::ValidationSeverity,
                                     &airborne_radar::session::ValidationIssue::severity>(
      issues, airborne_radar::session::ValidationSeverity::kError);
  EXPECT_FALSE(has_error);
}

TEST(ValidationUtilsTest, HasSeverityReturnsFalseWhenNoExpectedSeverity) {
  std::vector<airborne_radar::session::ValidationIssue> issues;
  issues.push_back(MakeIndexedIssue<airborne_radar::session::ValidationIssue,
                                    airborne_radar::session::ValidationSeverity,
                                    airborne_radar::session::ValidationCode,
                                    &airborne_radar::session::ValidationIssue::target_index>(
      airborne_radar::session::ValidationSeverity::kInfo,
      airborne_radar::session::ValidationCode::kUnknownExternalTargetId, 1U, "unknown id"));
  issues.push_back(MakeIndexedIssue<airborne_radar::session::ValidationIssue,
                                    airborne_radar::session::ValidationSeverity,
                                    airborne_radar::session::ValidationCode,
                                    &airborne_radar::session::ValidationIssue::target_index>(
      airborne_radar::session::ValidationSeverity::kWarning,
      airborne_radar::session::ValidationCode::kNegativeRcs, 2U, "negative rcs"));

  const bool has_error = HasSeverity<std::vector<airborne_radar::session::ValidationIssue>,
                                     airborne_radar::session::ValidationSeverity,
                                     &airborne_radar::session::ValidationIssue::severity>(
      issues, airborne_radar::session::ValidationSeverity::kError);
  EXPECT_FALSE(has_error);
}

TEST(ValidationUtilsTest, HasSeverityReturnsTrueWhenExpectedSeverityExists) {
  std::vector<electronic_surveillance_radar::session::EsrValidationIssue> issues;
  issues.push_back(
      MakeIndexedIssue<
          electronic_surveillance_radar::session::EsrValidationIssue,
          electronic_surveillance_radar::session::EsrValidationSeverity,
          electronic_surveillance_radar::session::EsrValidationCode,
          &electronic_surveillance_radar::session::EsrValidationIssue::emitter_index>(
          electronic_surveillance_radar::session::EsrValidationSeverity::kInfo,
          electronic_surveillance_radar::session::EsrValidationCode::kNone,
          static_cast<std::size_t>(-1), "info"));
  issues.push_back(
      MakeIndexedIssue<
          electronic_surveillance_radar::session::EsrValidationIssue,
          electronic_surveillance_radar::session::EsrValidationSeverity,
          electronic_surveillance_radar::session::EsrValidationCode,
          &electronic_surveillance_radar::session::EsrValidationIssue::emitter_index>(
          electronic_surveillance_radar::session::EsrValidationSeverity::kError,
          electronic_surveillance_radar::session::EsrValidationCode::kInvalidEmitterPri, 4U,
          "invalid pri"));

  const bool has_error =
      HasSeverity<std::vector<electronic_surveillance_radar::session::EsrValidationIssue>,
                  electronic_surveillance_radar::session::EsrValidationSeverity,
                  &electronic_surveillance_radar::session::EsrValidationIssue::severity>(
          issues, electronic_surveillance_radar::session::EsrValidationSeverity::kError);
  EXPECT_TRUE(has_error);
}

}  // namespace
}  // namespace validation
}  // namespace internal
}  // namespace oneq
