/**
 * @file esr_input_validation_test.cpp
 * @brief 验证 ESR 输入契约与基础校验规则。
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

#include "1q/electronic_surveillance_radar/model/EmitterHypothesis.h"
#include "1q/electronic_surveillance_radar/session/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/session/EsrInputValidation.h"

namespace electronic_surveillance_radar {
namespace session {
namespace {

/**
 * @brief 检查校验结果列表中是否包含指定编码。
 * @param[in] issues 校验结果列表。
 * @param[in] code 目标编码。
 * @return 若包含指定编码则返回 `true`。
 */
bool ContainsCode(const ValidationIssueList& issues, ValidationCode code) {
  for (std::size_t i = 0; i < issues.size(); ++i) {
    if (issues[i].code == code) {
      return true;
    }
  }
  return false;
}

/**
 * @brief 构造最小可用辐射源输入。
 * @return 最小可用辐射源输入。
 */
session::EsrSceneEmitter MakeValidEmitter() {
  session::EsrSceneEmitter emitter;
  emitter.emitter_id = "E-1";
  emitter.carrier_hz = 10.0e9;
  emitter.bandwidth_hz = 2.0e6;
  emitter.tx_power_w = 1.0e6;
  emitter.pulse_width_s = 2.0e-6f;
  emitter.pri_s = 2.0e-4f;
  emitter.pose.position_m.x = 1000.0f;
  return emitter;
}

template <typename T, typename = void>
struct HasTruthEmitterId : std::false_type {};

template <typename T>
struct HasTruthEmitterId<T, decltype((void)std::declval<T>().truth_emitter_id)> : std::true_type {};

static_assert(!HasTruthEmitterId<model::EmitterHypothesis>::value,
              "EmitterHypothesis must not expose truth emitter id");

TEST(EsrInputValidationTest, InvalidCycleDeltaTimeIsReportedAsError) {
  EsrCycleInput input;
  input.dt_sec = 0.0f;
  input.scene.push_back(MakeValidEmitter());

  const ValidationIssueList issues = ValidateEsrCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInvalidCycleDeltaTime));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EsrInputValidationTest, InvalidEmitterFrequencyIsReportedAsError) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  session::EsrSceneEmitter emitter = MakeValidEmitter();
  emitter.carrier_hz = 0.0;
  input.scene.push_back(emitter);

  const ValidationIssueList issues = ValidateEsrCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInvalidEmitterFrequency));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EsrInputValidationTest, EmptySceneInputIsAllowed) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;

  const ValidationIssueList issues = ValidateEsrCycleInput(input);

  EXPECT_FALSE(HasValidationError(issues));
  EXPECT_TRUE(issues.empty());
}

TEST(EsrInputValidationTest, NonFiniteEmitterNumericFieldIsReported) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  session::EsrSceneEmitter emitter = MakeValidEmitter();
  emitter.bandwidth_hz = std::numeric_limits<double>::infinity();
  input.scene.push_back(emitter);

  const ValidationIssueList issues = ValidateEsrCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kNonFiniteEmitterNumericField));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EsrInputValidationTest, NonFinitePlatformNumericFieldIsReported) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.platform_pose.attitude_deg.yaw_deg = std::numeric_limits<float>::infinity();
  input.scene.push_back(MakeValidEmitter());

  const ValidationIssueList issues = ValidateEsrCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kNonFinitePlatformNumericField));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EsrInputValidationTest, NonFiniteEmitterAttitudeIsReported) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  session::EsrSceneEmitter emitter = MakeValidEmitter();
  emitter.pose.attitude_deg.roll_deg = std::numeric_limits<float>::quiet_NaN();
  input.scene.push_back(emitter);

  const ValidationIssueList issues = ValidateEsrCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kNonFiniteEmitterNumericField));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EsrInputValidationTest, InvalidEmitterPulseWidthIsReportedAsError) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  session::EsrSceneEmitter emitter = MakeValidEmitter();
  emitter.pulse_width_s = 0.0f;
  input.scene.push_back(emitter);

  const ValidationIssueList issues = ValidateEsrCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInvalidEmitterPulseWidth));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EsrInputValidationTest, InvalidEmitterPriIsReportedAsError) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  session::EsrSceneEmitter emitter = MakeValidEmitter();
  emitter.pri_s = 0.0f;
  input.scene.push_back(emitter);

  const ValidationIssueList issues = ValidateEsrCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInvalidEmitterPri));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EsrInputValidationTest, EmitterPriLessThanPulseWidthIsReportedAsError) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  session::EsrSceneEmitter emitter = MakeValidEmitter();
  emitter.pulse_width_s = 3.0e-6f;
  emitter.pri_s = 1.0e-6f;
  input.scene.push_back(emitter);

  const ValidationIssueList issues = ValidateEsrCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kEmitterPriLessThanPulseWidth));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EsrInputValidationTest, InvalidEmitterBeamwidthIsReportedAsError) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  session::EsrSceneEmitter emitter = MakeValidEmitter();
  emitter.beam_state.az_beamwidth_deg = 0.0f;
  input.scene.push_back(emitter);

  const ValidationIssueList issues = ValidateEsrCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInvalidEmitterBeamwidth));
  EXPECT_TRUE(HasValidationError(issues));
}

}  // namespace
}  // namespace session

}  // namespace electronic_surveillance_radar
