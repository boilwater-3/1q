#include <gtest/gtest.h>

#include <string>

#include "batch_checks.h"
#include "batch_cli.h"

namespace {

TEST(BatchValidationSupportTest, ParsesExplicitSuiteScenarioAndOutput) {
  const char* raw[] = {"batch", "--suite", "sequence", "--scenario", "case-1",
                       "--output-dir", "/tmp/output"};
  char** argv = const_cast<char**>(raw);
  batch_validation::BatchCliOptions options;
  std::string error;
  ASSERT_TRUE(batch_validation::ParseBatchCli(7, argv, "/default", &options, &error)) << error;
  EXPECT_EQ(options.suite, batch_validation::ScenarioSuite::kSequence);
  EXPECT_EQ(options.scenario_id, "case-1");
  EXPECT_EQ(options.output_dir, "/tmp/output");
}

TEST(BatchValidationSupportTest, RejectsPositionalAndUnknownArguments) {
  const char* raw[] = {"batch", "/tmp/legacy"};
  char** argv = const_cast<char**>(raw);
  batch_validation::BatchCliOptions options;
  std::string error;
  EXPECT_FALSE(batch_validation::ParseBatchCli(2, argv, "/default", &options, &error));
  EXPECT_NE(error.find("unknown argument"), std::string::npos);
}

TEST(BatchValidationSupportTest, FiltersSweepAndSequenceSuitesExplicitly) {
  EXPECT_TRUE(batch_validation::IncludesSweep(batch_validation::ScenarioSuite::kSweep));
  EXPECT_FALSE(batch_validation::IncludesSequence(batch_validation::ScenarioSuite::kSweep));
  EXPECT_FALSE(batch_validation::IncludesSweep(batch_validation::ScenarioSuite::kSequence));
  EXPECT_TRUE(batch_validation::IncludesSequence(batch_validation::ScenarioSuite::kSequence));
  EXPECT_TRUE(batch_validation::IncludesSweep(batch_validation::ScenarioSuite::kAll));
  EXPECT_TRUE(batch_validation::IncludesSequence(batch_validation::ScenarioSuite::kAll));
}

TEST(BatchValidationSupportTest, CountsOnlyFailedChecksAtRequestedSeverity) {
  batch_validation::ContractCheckCollector checks;
  checks.Add("s", "phase", 1U, "pass", "1", "1", true);
  checks.Add("s", "phase", 2U, "warn", "1", "0", false,
             batch_validation::Severity::kWarning);
  checks.Add("s", "phase", 3U, "error", "1", "0", false);
  EXPECT_EQ(checks.size(), 3U);
  EXPECT_EQ(checks.FailureCount(), 1U);
  EXPECT_EQ(checks.FailureCount(batch_validation::Severity::kWarning), 2U);
}

}  // namespace
