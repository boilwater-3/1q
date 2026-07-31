#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarInputValidation.h"

namespace sar {
namespace session {
namespace {

TEST(SarInputValidationTest, ValidInputHasNoErrors) {
  SarCycleInput input;
  input.dt_sec = 1.0f;
  input.platform.latitude_deg = 30.0;
  input.platform.altitude_m = 8000.0;

  const ValidationIssueList issues = ValidateSarCycleInput(input);
  EXPECT_FALSE(HasValidationError(issues));
}

TEST(SarInputValidationTest, NonPositiveDtIsError) {
  SarCycleInput input;
  input.dt_sec = 0.0f;

  const ValidationIssueList issues = ValidateSarCycleInput(input);
  bool found = false;
  for (const ValidationIssue& issue : issues) {
    if (issue.code == ValidationCode::kInvalidCycleDeltaTime &&
        issue.severity == ValidationSeverity::kError) {
      found = true;
    }
  }
  EXPECT_TRUE(found);
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(SarInputValidationTest, NonFiniteDtIsError) {
  SarCycleInput input;
  input.dt_sec = std::numeric_limits<float>::quiet_NaN();

  const ValidationIssueList issues = ValidateSarCycleInput(input);
  bool found = false;
  for (const ValidationIssue& issue : issues) {
    if (issue.code == ValidationCode::kNonFiniteCycleDeltaTime) {
      found = true;
    }
  }
  EXPECT_TRUE(found);
}

TEST(SarInputValidationTest, NonFinitePlatformFieldIsError) {
  SarCycleInput input;
  input.dt_sec = 1.0f;
  input.platform.latitude_deg = std::numeric_limits<double>::infinity();

  const ValidationIssueList issues = ValidateSarCycleInput(input);
  bool found = false;
  for (const ValidationIssue& issue : issues) {
    if (issue.code == ValidationCode::kNonFinitePlatformField) {
      found = true;
    }
  }
  EXPECT_TRUE(found);
}

TEST(SarInputValidationTest, NonFiniteTargetFieldIsError) {
  SarCycleInput input;
  input.dt_sec = 1.0f;
  SarPointTarget target;
  target.radar_cross_section_dbsm = std::numeric_limits<double>::quiet_NaN();
  input.point_targets.push_back(target);

  const ValidationIssueList issues = ValidateSarCycleInput(input);
  bool found = false;
  for (const ValidationIssue& issue : issues) {
    if (issue.code == ValidationCode::kNonFiniteTargetField &&
        issue.location.entity_index == 0) {
      found = true;
    }
  }
  EXPECT_TRUE(found);
}

TEST(SarInputValidationTest, NonContiguousPulseIdIsError) {
  SarCycleInput input;
  input.dt_sec = 1.0f;

  SarRawIqFrame::PulseState pulse1;
  pulse1.pulse_id = 1U;
  pulse1.time_s = 0.0;
  input.raw_iq.pulse_states.push_back(pulse1);

  SarRawIqFrame::PulseState pulse2;
  pulse2.pulse_id = 5U;  // 不连续
  pulse2.time_s = 1.0;
  input.raw_iq.pulse_states.push_back(pulse2);

  const ValidationIssueList issues = ValidateSarCycleInput(input);
  bool found = false;
  for (const ValidationIssue& issue : issues) {
    if (issue.code == ValidationCode::kInvalidPulseSequence) {
      found = true;
    }
  }
  EXPECT_TRUE(found);
}

TEST(SarInputValidationTest, EmptyPulsesProduceNoPulseErrors) {
  SarCycleInput input;
  input.dt_sec = 1.0f;
  // raw_iq 保持默认空值

  const ValidationIssueList issues = ValidateSarCycleInput(input);
  for (const ValidationIssue& issue : issues) {
    EXPECT_NE(issue.code, ValidationCode::kNonFinitePulseField);
    EXPECT_NE(issue.code, ValidationCode::kInvalidPulseSequence);
  }
}

}  // namespace
}  // namespace session
}  // namespace sar
