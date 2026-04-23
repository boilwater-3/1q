/**
 * @file validation_utils_test.cpp
 * @brief 验证校验通用工具的有限值、问题构造与严重级别扫描行为。
 */

#include <gtest/gtest.h>

#include <limits>
#include <cstdint>
#include <string>
#include <vector>

#include "common/validation/ValidationUtils.h"

namespace oneq {
namespace internal {
namespace validation {
namespace {

enum class TestSeverity {
  kInfo = 0,
  kWarning,
  kError
};

enum class TestCode {
  kNone = 0,
  kA,
  kB
};

struct TestIssue {
  TestSeverity severity{TestSeverity::kInfo};
  TestCode code{TestCode::kNone};
  std::size_t entity_index{static_cast<std::size_t>(-1)};
  std::string message{};
};

TEST(ValidationUtilsTest, IsFiniteDistinguishesFiniteNanAndInfinity) {
  EXPECT_TRUE(IsFinite(1.0f));
  EXPECT_TRUE(IsFinite(3.5));
  EXPECT_FALSE(IsFinite(std::numeric_limits<float>::quiet_NaN()));
  EXPECT_FALSE(IsFinite(std::numeric_limits<double>::infinity()));
}

TEST(ValidationUtilsTest, MakeIndexedIssueSupportsSizeTIndexField) {
  const TestIssue issue = MakeIndexedIssue<TestIssue, TestSeverity, TestCode,
                                           &TestIssue::entity_index>(
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

  const Int32Issue issue = MakeIndexedIssue<Int32Issue, TestSeverity, TestCode,
                                            &Int32Issue::entity_index>(
      TestSeverity::kError, TestCode::kB, static_cast<std::int32_t>(5), "issue-b");
  EXPECT_EQ(issue.severity, TestSeverity::kError);
  EXPECT_EQ(issue.code, TestCode::kB);
  EXPECT_EQ(issue.entity_index, 5);
  EXPECT_EQ(issue.message, "issue-b");
}

TEST(ValidationUtilsTest, HasSeverityReturnsFalseForEmptyList) {
  const std::vector<TestIssue> issues;
  const bool has_error =
      HasSeverity<std::vector<TestIssue>, TestSeverity, &TestIssue::severity>(
          issues, TestSeverity::kError);
  EXPECT_FALSE(has_error);
}

TEST(ValidationUtilsTest, HasSeverityReturnsFalseWhenNoExpectedSeverity) {
  std::vector<TestIssue> issues;
  issues.push_back(MakeIndexedIssue<TestIssue, TestSeverity, TestCode,
                                    &TestIssue::entity_index>(
      TestSeverity::kInfo, TestCode::kA, 1U, "info"));
  issues.push_back(MakeIndexedIssue<TestIssue, TestSeverity, TestCode,
                                    &TestIssue::entity_index>(
      TestSeverity::kWarning, TestCode::kB, 2U, "warning"));

  const bool has_error =
      HasSeverity<std::vector<TestIssue>, TestSeverity, &TestIssue::severity>(
          issues, TestSeverity::kError);
  EXPECT_FALSE(has_error);
}

TEST(ValidationUtilsTest, HasSeverityReturnsTrueWhenExpectedSeverityExists) {
  std::vector<TestIssue> issues;
  issues.push_back(MakeIndexedIssue<TestIssue, TestSeverity, TestCode,
                                    &TestIssue::entity_index>(
      TestSeverity::kInfo, TestCode::kNone, static_cast<std::size_t>(-1), "info"));
  issues.push_back(MakeIndexedIssue<TestIssue, TestSeverity, TestCode,
                                    &TestIssue::entity_index>(
      TestSeverity::kError, TestCode::kB, 4U, "error"));

  const bool has_error =
      HasSeverity<std::vector<TestIssue>, TestSeverity, &TestIssue::severity>(
          issues, TestSeverity::kError);
  EXPECT_TRUE(has_error);
}

}  // namespace
}  // namespace validation
}  // namespace internal
}  // namespace oneq
