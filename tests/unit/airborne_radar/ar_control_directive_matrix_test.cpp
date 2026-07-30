/**
 * @file ar_control_directive_matrix_test.cpp
 * @brief 逐 directive 的契约矩阵测试。
 *
 * 这是「控制效果传播闭包」基线测试（Stage B 第 1 步）。
 * 每条 ControlDirectiveType 必须有一条矩阵用例，断言其在 ControlReducer::Reduce 后
 * 写入 ArControlProfile 的预期字段，以及标量合法性。
 * 本测试锁定现状：通过即证明 reducer 的 directive→profile 映射当前是自洽的，
 * 为后续 C2/C5 重构提供行为基线。
 */

#include <gtest/gtest.h>

#include <vector>

#include "1q/airborne_radar/session/ArControlProfile.h"
#include "1q/airborne_radar/session/ControlDirective.h"
#include "airborne_radar/decision/ControlReducer.h"

namespace airborne_radar {
namespace {

/**
 * @brief 矩阵用例的期望：单条 directive 应在 profile 上产生的字段变化。
 *
 * 字段名以 ArControlProfile 成员命名为准；标量 directive 额外携带预期值。
 */
struct DirectiveMatrixExpectation {
  session::ControlDirectiveType type;
  bool expects_value;        // 是否要求携带 requested_value（标量语义）
  float requested_value;     // 标量 directive 的合法值
  std::string profile_field; // 期望被置位的 profile 字段名（用于错误信息）
  float expected_scalar;     // 期望的标量值（仅 power/dwell/burnthrough）

  // 构造一个带标量的 directive（power/dwell/burnthrough）。
  static DirectiveMatrixExpectation Scalar(session::ControlDirectiveType t, float value,
                                           const std::string& field, float expected) {
    DirectiveMatrixExpectation e;
    e.type = t;
    e.expects_value = true;
    e.requested_value = value;
    e.profile_field = field;
    e.expected_scalar = expected;
    return e;
  }

  // 构造一个布尔 directive（无标量）。
  static DirectiveMatrixExpectation Flag(session::ControlDirectiveType t,
                                         const std::string& field) {
    DirectiveMatrixExpectation e;
    e.type = t;
    e.expects_value = false;
    e.requested_value = 0.0f;
    e.profile_field = field;
    e.expected_scalar = 0.0f;
    return e;
  }
};

/**
 * @brief 把单条 directive 喂给 reducer，返回生成的 profile。
 */
session::ArControlProfile ReduceSingle(const DirectiveMatrixExpectation& expectation) {
  decision::ControlReducer reducer({});
  session::ControlDirective directive =
      expectation.expects_value
          ? session::ControlDirective(expectation.type,
                                      session::ControlDirectiveSource::SURVIVABILITY,
                                      expectation.requested_value)
          : session::ControlDirective(expectation.type,
                                      session::ControlDirectiveSource::SURVIVABILITY);
  session::TacticalProposal proposal(directive, 50, "");
  std::vector<session::TacticalProposal> proposals{proposal};
  session::ArControlProfile previous;
  return reducer.Reduce(previous, proposals).profile;
}

/**
 * @brief 断言指定 profile 字段为 true（布尔 directive 期望）。
 */
void ExpectBoolField(const session::ArControlProfile& profile,
                     const std::string& field) {
  if (field == "enable_lpi_power_control") {
    EXPECT_TRUE(profile.enable_lpi_power_control) << field;
  } else if (field == "enable_lpi_beamforming") {
    EXPECT_TRUE(profile.enable_lpi_beamforming) << field;
  } else if (field == "enable_agility_frequency") {
    EXPECT_TRUE(profile.enable_agility_frequency) << field;
  } else if (field == "enable_sidelobe_canceller") {
    EXPECT_TRUE(profile.enable_sidelobe_canceller) << field;
  } else if (field == "enable_adaptive_beamforming") {
    EXPECT_TRUE(profile.enable_adaptive_beamforming) << field;
  } else if (field == "enable_eccm_rejitter") {
    EXPECT_TRUE(profile.enable_eccm_rejitter) << field;
  } else if (field == "enable_anti_rgpo_leading_edge") {
    EXPECT_TRUE(profile.enable_anti_rgpo_leading_edge) << field;
  } else if (field == "enable_anti_vgpo_acceleration_bound") {
    EXPECT_TRUE(profile.enable_anti_vgpo_acceleration_bound) << field;
  } else if (field == "enable_anti_false_target_discrimination") {
    EXPECT_TRUE(profile.enable_anti_false_target_discrimination) << field;
  } else {
    FAIL() << "未知 profile 布尔字段: " << field;
  }
}

/**
 * @brief 断言指定 profile 标量字段等于期望值。
 */
void ExpectScalarField(const session::ArControlProfile& profile,
                       const std::string& field, float expected) {
  if (field == "lpi_power_scale") {
    EXPECT_FLOAT_EQ(profile.lpi_power_scale, expected) << field;
  } else if (field == "lpi_dwell_scale") {
    EXPECT_FLOAT_EQ(profile.lpi_dwell_scale, expected) << field;
  } else if (field == "eccm_burnthrough_gain") {
    EXPECT_FLOAT_EQ(profile.eccm_burnthrough_gain, expected) << field;
  } else {
    FAIL() << "未知 profile 标量字段: " << field;
  }
}

class ControlDirectiveMatrixTest : public ::testing::TestWithParam<DirectiveMatrixExpectation> {};

TEST_P(ControlDirectiveMatrixTest, DirectiveMapsToExpectedProfileField) {
  const DirectiveMatrixExpectation& expectation = GetParam();
  const session::ArControlProfile profile = ReduceSingle(expectation);

  if (expectation.expects_value) {
    ExpectScalarField(profile, expectation.profile_field, expectation.expected_scalar);
  } else {
    ExpectBoolField(profile, expectation.profile_field);
  }
}

INSTANTIATE_TEST_SUITE_P(
    PerDirective, ControlDirectiveMatrixTest,
    ::testing::Values(
        // LPI 标量域：power ∈ (0,1]，dwell ∈ [0.25,1]，burnthrough ∈ (1,2]。
        DirectiveMatrixExpectation::Scalar(
            session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION, 0.5f,
            "lpi_power_scale", 0.5f),
        DirectiveMatrixExpectation::Scalar(
            session::ControlDirectiveType::REQUEST_LPI_DWELL, 0.7f,
            "lpi_dwell_scale", 0.7f),
        DirectiveMatrixExpectation::Scalar(
            session::ControlDirectiveType::REQUEST_ECCM_BURNTHROUGH_GAIN, 1.5f,
            "eccm_burnthrough_gain", 1.5f),
        // 布尔域（不应携带标量）。
        DirectiveMatrixExpectation::Flag(
            session::ControlDirectiveType::REQUEST_LPI_BEAMFORMING, "enable_lpi_beamforming"),
        DirectiveMatrixExpectation::Flag(
            session::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER,
            "enable_sidelobe_canceller"),
        DirectiveMatrixExpectation::Flag(
            session::ControlDirectiveType::REQUEST_ENABLE_ADAPTIVE_BEAMFORMING,
            "enable_adaptive_beamforming"),
        DirectiveMatrixExpectation::Flag(
            session::ControlDirectiveType::REQUEST_AGILITY_FREQUENCY,
            "enable_agility_frequency"),
        DirectiveMatrixExpectation::Flag(
            session::ControlDirectiveType::REQUEST_ECCM_REJITTER, "enable_eccm_rejitter"),
        DirectiveMatrixExpectation::Flag(
            session::ControlDirectiveType::REQUEST_ANTI_RGPO_LEADING_EDGE,
            "enable_anti_rgpo_leading_edge"),
        DirectiveMatrixExpectation::Flag(
            session::ControlDirectiveType::REQUEST_ANTI_VGPO_ACCELERATION_BOUND,
            "enable_anti_vgpo_acceleration_bound"),
        DirectiveMatrixExpectation::Flag(
            session::ControlDirectiveType::REQUEST_ANTI_FALSE_TARGET_DISCRIMINATION,
            "enable_anti_false_target_discrimination")));

// 布尔 directive 携带标量应被拒绝（profile 字段保持关闭）。
TEST(ControlDirectiveMatrixTest, BooleanDirectiveWithValueIsRejected) {
  decision::ControlReducer reducer({});
  session::ControlDirective directive(
      session::ControlDirectiveType::REQUEST_ENABLE_SIDELOBE_CANCELLER,
      session::ControlDirectiveSource::SURVIVABILITY, 0.5f);
  session::TacticalProposal proposal(directive, 50, "");
  session::ArControlProfile previous;
  const auto result = reducer.Reduce(previous, {proposal});
  EXPECT_FALSE(result.profile.enable_sidelobe_canceller);
  EXPECT_EQ(result.rejected_directives.size(), 1U);
}

// 标量 directive 缺值应被拒绝。
TEST(ControlDirectiveMatrixTest, ScalarDirectiveMissingValueIsRejected) {
  decision::ControlReducer reducer({});
  session::ControlDirective directive(
      session::ControlDirectiveType::REQUEST_LPI_POWER_REDUCTION,
      session::ControlDirectiveSource::SURVIVABILITY);
  session::TacticalProposal proposal(directive, 50, "");
  session::ArControlProfile previous;
  const auto result = reducer.Reduce(previous, {proposal});
  EXPECT_FALSE(result.profile.enable_lpi_power_control);
  EXPECT_EQ(result.rejected_directives.size(), 1U);
}

}  // namespace
}  // namespace airborne_radar
