#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "1q/sbirs_sensor/session/SbirsCycleInputAdapter.h"
#include "1q/sbirs_sensor/session/SbirsInputValidation.h"
#include "1q/sbirs_sensor/session/SbirsIssueCodes.h"

namespace {

sbirs_sensor::session::SbirsVector3M Vector(double x, double y, double z) {
  sbirs_sensor::session::SbirsVector3M value;
  value.x = x;
  value.y = y;
  value.z = z;
  return value;
}

TEST(SbirsInputValidationTest, RejectsMissingSatellitePosition) {
  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .Build();

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_TRUE(sbirs_sensor::session::HasValidationError(issues));
  // Q-2 审查修复：error 级校验问题统一 phase=kInputValidation
  // （HasValidationError 依赖该判定，防误改时拒绝语义静默翻转）。
  for (const auto& issue : issues) {
    if (issue.severity == sbirs_sensor::session::SbirsIssueSeverity::kError) {
      EXPECT_EQ(issue.phase, sbirs_sensor::session::SbirsIssuePhase::kInputValidation);
    }
  }
}

TEST(SbirsInputValidationTest, AcceptsMinimalValidScene) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 10U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e4;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(target)
          .Build();

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_FALSE(sbirs_sensor::session::HasValidationError(issues));
}

TEST(SbirsInputValidationTest, RejectsFiniteDomainFlagIdAndEnvironmentMatrix) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 10U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e4;
  const sbirs_sensor::session::SbirsCycleInput valid =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(target)
          .Build();

  std::vector<sbirs_sensor::session::SbirsCycleInput> invalid_inputs;
  invalid_inputs.push_back(valid);
  invalid_inputs.back().satellite_position_ecef_m = Vector(0.0, 0.0, 0.0);
  invalid_inputs.push_back(valid);
  invalid_inputs.back().scene.front().radiant_intensity_w_per_sr =
      std::numeric_limits<double>::quiet_NaN();
  invalid_inputs.push_back(valid);
  invalid_inputs.back().scene.front().radiant_intensity_w_per_sr =
      std::numeric_limits<double>::infinity();
  invalid_inputs.push_back(valid);
  invalid_inputs.back().scene.front().radiant_intensity_w_per_sr = -1.0;
  invalid_inputs.push_back(valid);
  invalid_inputs.back().scene.front().target_id = 0U;
  invalid_inputs.push_back(valid);
  invalid_inputs.back().scene.push_back(invalid_inputs.back().scene.front());
  invalid_inputs.push_back(valid);
  invalid_inputs.back().scene.front().velocity_ecef_m_per_s.x = 1.0;
  invalid_inputs.push_back(valid);
  invalid_inputs.back().scene.front().has_velocity_ecef_m_per_s = true;
  invalid_inputs.back().scene.front().velocity_ecef_m_per_s.y =
      std::numeric_limits<double>::quiet_NaN();

  for (std::size_t i = 0; i < invalid_inputs.size(); ++i) {
    const sbirs_sensor::session::SbirsIssueList issues =
        sbirs_sensor::session::ValidateSbirsCycleInput(invalid_inputs[i], 10.0f);
    EXPECT_TRUE(sbirs_sensor::session::HasValidationError(issues)) << "case " << i;
    // Q-2 审查修复：error 级校验问题统一 phase=kInputValidation（同 RejectsMissingSatellitePosition）。
    for (const auto& issue : issues) {
      if (issue.severity == sbirs_sensor::session::SbirsIssueSeverity::kError) {
        EXPECT_EQ(issue.phase, sbirs_sensor::session::SbirsIssuePhase::kInputValidation);
      }
    }
  }
}

TEST(SbirsInputValidationTest, AcceptsDtSecWithinFrameRateBound) {
  // frame_rate_hz=10 → max_dt = 10/10 = 1.0s; dt_sec=0.5 is within range.
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e4;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(0.5f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(target)
          .Build();

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_FALSE(sbirs_sensor::session::HasValidationError(issues));
}


TEST(SbirsInputValidationTest, RejectsMissingOrInvalidUtcJulianDay) {
  // 2026-08 正式变更：ECI 输出参考系要求每周期携带 UTC 儒略日（缺失=0/非有限/
  // 非正 → 校验拒绝，code sbirs.validation.invalid_utc_julian_day）。
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 10U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e4;

  const sbirs_sensor::session::SbirsCycleInput valid =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(target)
          .Build();

  std::vector<sbirs_sensor::session::SbirsCycleInput> invalid_inputs;
  invalid_inputs.push_back(valid);
  invalid_inputs.back().utc_julian_day = 0.0;  // 缺失（默认值）
  invalid_inputs.push_back(valid);
  invalid_inputs.back().utc_julian_day = -1.0;
  invalid_inputs.push_back(valid);
  invalid_inputs.back().utc_julian_day = std::numeric_limits<double>::quiet_NaN();

  for (std::size_t i = 0; i < invalid_inputs.size(); ++i) {
    const sbirs_sensor::session::SbirsIssueList issues =
        sbirs_sensor::session::ValidateSbirsCycleInput(invalid_inputs[i], 10.0f);
    EXPECT_TRUE(sbirs_sensor::session::HasValidationError(issues)) << "case " << i;
    bool found_code = false;
    for (const auto& issue : issues) {
      EXPECT_EQ(issue.phase, sbirs_sensor::session::SbirsIssuePhase::kInputValidation);
      if (issue.code == sbirs_sensor::session::codes::kInvalidUtcJulianDay) {
        found_code = true;
      }
    }
    EXPECT_TRUE(found_code) << "case " << i << " missing invalid_utc_julian_day code";
  }
}

TEST(SbirsInputValidationTest, RejectsDtSecExceedingFrameRateBound) {
  // frame_rate_hz=10 → max_dt = 10/10 = 1.0s; dt_sec=2.0 exceeds the bound.
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e4;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(2.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(target)
          .Build();

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_TRUE(sbirs_sensor::session::HasValidationError(issues));
}

TEST(SbirsInputValidationTest, AcceptsDtSecAtExactFrameRateBound) {
  // frame_rate_hz=10 → max_dt = 10/10 = 1.0s; dt_sec=1.0 is exactly at the boundary.
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e4;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(target)
          .Build();

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_FALSE(sbirs_sensor::session::HasValidationError(issues));
}

TEST(SbirsInputValidationTest, RejectsDtSecJustAboveFrameRateBound) {
  // frame_rate_hz=10 → max_dt = 1.0s; dt_sec=1.001 is just above.
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e4;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.001f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(target)
          .Build();

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_TRUE(sbirs_sensor::session::HasValidationError(issues));
}

TEST(SbirsInputValidationTest, HigherFrameRateAllowsTighterDtSec) {
  // frame_rate_hz=20 → max_dt = 10/20 = 0.5s; dt_sec=0.8 should fail.
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e4;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(0.8f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(target)
          .Build();

  // At 20Hz, max_dt=0.5s → 0.8s should fail.
  const sbirs_sensor::session::SbirsIssueList issues_20hz =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 20.0f);
  EXPECT_TRUE(sbirs_sensor::session::HasValidationError(issues_20hz));

  // At 10Hz, max_dt=1.0s → 0.8s should pass.
  const sbirs_sensor::session::SbirsIssueList issues_10hz =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_FALSE(sbirs_sensor::session::HasValidationError(issues_10hz));
}

// 卫星速度必填（合同指标 2）：缺失即拒绝，code 为 invalid_satellite_velocity。
TEST(SbirsInputValidationTest, RejectsMissingSatelliteVelocity) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e4;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          // 故意不调用 WithSatelliteVelocity。
          .AddTarget(target)
          .Build();

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_TRUE(sbirs_sensor::session::HasValidationError(issues));
  bool found_code = false;
  for (const auto& issue : issues) {
    EXPECT_EQ(issue.phase, sbirs_sensor::session::SbirsIssuePhase::kInputValidation);
    if (issue.code == sbirs_sensor::session::codes::kInvalidSatelliteVelocity) {
      found_code = true;
      EXPECT_EQ(issue.severity, sbirs_sensor::session::SbirsIssueSeverity::kError);
    }
  }
  EXPECT_TRUE(found_code);
}

// ECEF 零速度合法（GEO 卫星在 ECEF 中静止）。
TEST(SbirsInputValidationTest, AcceptsZeroSatelliteVelocity) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e4;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{}).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(target)
          .Build();

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_FALSE(sbirs_sensor::session::HasValidationError(issues));
}

TEST(SbirsInputValidationTest, RejectsNonFiniteSatelliteVelocity) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e4;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(Vector(0.0, std::numeric_limits<double>::quiet_NaN(), 0.0)).WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(target)
          .Build();

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_TRUE(sbirs_sensor::session::HasValidationError(issues));
  bool found_code = false;
  for (const auto& issue : issues) {
    if (issue.code == sbirs_sensor::session::codes::kInvalidSatelliteVelocity) {
      found_code = true;
    }
  }
  EXPECT_TRUE(found_code);
}

TEST(SbirsInputValidationTest, RejectsMissingSatelliteAttitude) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e4;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          // 故意不调用 WithSatelliteAttitude。
          .AddTarget(target)
          .Build();

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_TRUE(sbirs_sensor::session::HasValidationError(issues));
  bool found_code = false;
  for (const auto& issue : issues) {
    EXPECT_EQ(issue.phase, sbirs_sensor::session::SbirsIssuePhase::kInputValidation);
    if (issue.code == sbirs_sensor::session::codes::kInvalidSatelliteAttitude) {
      found_code = true;
      EXPECT_EQ(issue.severity, sbirs_sensor::session::SbirsIssueSeverity::kError);
    }
  }
  EXPECT_TRUE(found_code);
}

// 零欧拉姿态合法（体轴对齐 ECI）。
TEST(SbirsInputValidationTest, AcceptsZeroSatelliteAttitude) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e4;

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          .WithSatelliteAttitude(sbirs_sensor::session::SbirsEulerAnglesDeg{})
          .AddTarget(target)
          .Build();

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_FALSE(sbirs_sensor::session::HasValidationError(issues));
}

TEST(SbirsInputValidationTest, RejectsNonFiniteSatelliteAttitude) {
  sbirs_sensor::session::SbirsSceneTarget target;
  target.target_id = 1U;
  target.position_ecef_m = Vector(8000000.0, 0.0, 0.0);
  target.radiant_intensity_w_per_sr = 1.0e4;

  sbirs_sensor::session::SbirsEulerAnglesDeg attitude;
  attitude.pitch_deg = std::numeric_limits<double>::quiet_NaN();

  const sbirs_sensor::session::SbirsCycleInput input =
      sbirs_sensor::session::SbirsCycleInputBuilder()
          .WithCycleIndex(1U)
          .WithDeltaTimeSec(1.0f)
          .WithUtcJulianDay(2451544.2230698913)  // GMST≈0：ECI≡ECEF
          .WithSatellitePosition(Vector(7000000.0, 0.0, 0.0))
          .WithSatelliteVelocity(sbirs_sensor::session::SbirsVector3M{})
          .WithSatelliteAttitude(attitude)
          .AddTarget(target)
          .Build();

  const sbirs_sensor::session::SbirsIssueList issues =
      sbirs_sensor::session::ValidateSbirsCycleInput(input, 10.0f);
  EXPECT_TRUE(sbirs_sensor::session::HasValidationError(issues));
  bool found_code = false;
  for (const auto& issue : issues) {
    if (issue.code == sbirs_sensor::session::codes::kInvalidSatelliteAttitude) {
      found_code = true;
    }
  }
  EXPECT_TRUE(found_code);
}

}  // namespace
