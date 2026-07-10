/**
 * @file validation_utils_test.cpp
 * @brief 验证校验通用工具的有限值、问题构造与严重级别扫描行为。
 */

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "common/validation/ValidationUtils.h"

namespace oneq {
namespace common {
namespace validation {
namespace {

enum class TestSeverity { kInfo = 0, kWarning, kError };

enum class TestCode { kNone = 0, kA, kB };

enum class TestLocationKind { kScene = 0, kTarget };

struct TestLocation {
  TestLocationKind kind{TestLocationKind::kScene};
  std::size_t entity_index{static_cast<std::size_t>(-1)};
};

struct TestIssue {
  TestSeverity severity{TestSeverity::kInfo};
  TestCode code{TestCode::kNone};
  TestLocation location{};
  std::string field{};
  std::size_t entity_index{static_cast<std::size_t>(-1)};
  std::string message{};
};

TEST(ValidationUtilsTest, IsFiniteDistinguishesFiniteNanAndInfinity) {
  EXPECT_TRUE(IsFinite(1.0f));
  EXPECT_TRUE(IsFinite(3.5));
  EXPECT_FALSE(IsFinite(std::numeric_limits<float>::quiet_NaN()));
  EXPECT_FALSE(IsFinite(std::numeric_limits<double>::infinity()));
}

TEST(ValidationUtilsTest, IsRatio01IncludesBoundariesAndRejectsInvalidValues) {
  EXPECT_TRUE(IsRatio01(0.0));
  EXPECT_TRUE(IsRatio01(0.5));
  EXPECT_TRUE(IsRatio01(1.0));
  EXPECT_FALSE(IsRatio01(-0.1));
  EXPECT_FALSE(IsRatio01(1.1));
  EXPECT_FALSE(IsRatio01(std::numeric_limits<double>::quiet_NaN()));
}

TEST(ValidationUtilsTest, MakeLocatedIssueFillsLocationFieldAndMessage) {
  const TestIssue issue = MakeLocatedIssue<TestIssue, TestLocation>(
      TestSeverity::kError, TestCode::kB, TestLocationKind::kTarget, 7U, "target.confidence",
      "out of range");

  EXPECT_EQ(issue.severity, TestSeverity::kError);
  EXPECT_EQ(issue.code, TestCode::kB);
  EXPECT_EQ(issue.location.kind, TestLocationKind::kTarget);
  EXPECT_EQ(issue.location.entity_index, 7U);
  EXPECT_EQ(issue.field, "target.confidence");
  EXPECT_EQ(issue.message, "out of range");
}

TEST(ValidationUtilsTest, MakeIndexedIssueSupportsSizeTIndexField) {
  const TestIssue issue =
      MakeIndexedIssue<TestIssue, TestSeverity, TestCode, &TestIssue::entity_index>(
          TestSeverity::kWarning, TestCode::kA, 3U, "issue-a");

  EXPECT_EQ(issue.severity, TestSeverity::kWarning);
  EXPECT_EQ(issue.code, TestCode::kA);
  EXPECT_EQ(issue.entity_index, 3U);
  EXPECT_EQ(issue.message, "issue-a");
}

TEST(ValidationUtilsTest, MakeIndexedIssueSupportsInt32IndexField) {
  struct Int32Issue {
    TestSeverity severity{TestSeverity::kInfo};
    TestCode code{TestCode::kNone};
    std::int32_t entity_index{0};
    std::string message{};
  };

  const Int32Issue issue =
      MakeIndexedIssue<Int32Issue, TestSeverity, TestCode, &Int32Issue::entity_index>(
          TestSeverity::kError, TestCode::kB, static_cast<std::int32_t>(5), "issue-b");
  EXPECT_EQ(issue.severity, TestSeverity::kError);
  EXPECT_EQ(issue.code, TestCode::kB);
  EXPECT_EQ(issue.entity_index, 5);
  EXPECT_EQ(issue.message, "issue-b");
}

TEST(ValidationUtilsTest, HasSeverityReturnsFalseForEmptyList) {
  const std::vector<TestIssue> issues;
  const bool has_error = HasSeverity<std::vector<TestIssue>, TestSeverity, &TestIssue::severity>(
      issues, TestSeverity::kError);
  EXPECT_FALSE(has_error);
}

TEST(ValidationUtilsTest, HasSeverityReturnsFalseWhenNoExpectedSeverity) {
  std::vector<TestIssue> issues;
  issues.push_back(MakeIndexedIssue<TestIssue, TestSeverity, TestCode, &TestIssue::entity_index>(
      TestSeverity::kInfo, TestCode::kA, 1U, "info"));
  issues.push_back(MakeIndexedIssue<TestIssue, TestSeverity, TestCode, &TestIssue::entity_index>(
      TestSeverity::kWarning, TestCode::kB, 2U, "warning"));

  const bool has_error = HasSeverity<std::vector<TestIssue>, TestSeverity, &TestIssue::severity>(
      issues, TestSeverity::kError);
  EXPECT_FALSE(has_error);
}

TEST(ValidationUtilsTest, HasSeverityReturnsTrueWhenExpectedSeverityExists) {
  std::vector<TestIssue> issues;
  issues.push_back(MakeIndexedIssue<TestIssue, TestSeverity, TestCode, &TestIssue::entity_index>(
      TestSeverity::kInfo, TestCode::kNone, static_cast<std::size_t>(-1), "info"));
  issues.push_back(MakeIndexedIssue<TestIssue, TestSeverity, TestCode, &TestIssue::entity_index>(
      TestSeverity::kError, TestCode::kB, 4U, "error"));

  const bool has_error = HasSeverity<std::vector<TestIssue>, TestSeverity, &TestIssue::severity>(
      issues, TestSeverity::kError);
  EXPECT_TRUE(has_error);
}

}  // namespace
}  // namespace validation
}  // namespace common
}  // namespace oneq
