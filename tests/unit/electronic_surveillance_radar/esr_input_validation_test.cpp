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

#include "1q/electronic_surveillance_radar/session/EmitterHypothesis.h"
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
  emitter.emitter_id = 1001U;
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

static_assert(!HasTruthEmitterId<session::EmitterHypothesis>::value,
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

TEST(EsrInputValidationTest, NonFinitePlatformAltitudeIsReported) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.platform_altitude_m = std::numeric_limits<float>::quiet_NaN();
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

// =============================================================================
// 环境观测校验（ValidateEnvironmentObservation 分支）
// =============================================================================

TEST(EsrInputValidationTest, NonFiniteCycleDeltaTimeIsReported) {
  EsrCycleInput input;
  input.dt_sec = std::numeric_limits<float>::quiet_NaN();
  input.scene.push_back(MakeValidEmitter());

  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kNonFiniteCycleDeltaTime));
}

TEST(EsrInputValidationTest, InvalidSpectrumOccupancyRatioIsReported) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.environment.spectrum_occupancy_ratio = 1.5f;  // 超出 [0,1]
  input.scene.push_back(MakeValidEmitter());

  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInvalidEnvironmentObservation));
}

TEST(EsrInputValidationTest, InvalidHumidityRatioIsReported) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.environment.atmospheric_observation.relative_humidity_ratio = -0.1f;
  input.scene.push_back(MakeValidEmitter());

  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInvalidEnvironmentObservation));
}

TEST(EsrInputValidationTest, NegativePrecipitationRateIsReported) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.environment.atmospheric_observation.precipitation_rate_mmph = -1.0f;
  input.scene.push_back(MakeValidEmitter());

  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInvalidEnvironmentObservation));
}

TEST(EsrInputValidationTest, NonPositiveVisibilityIsReported) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.environment.atmospheric_observation.visibility_km = 0.0f;
  input.scene.push_back(MakeValidEmitter());

  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInvalidEnvironmentObservation));
}

TEST(EsrInputValidationTest, NonFiniteVisibilityIsReported) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.environment.atmospheric_observation.visibility_km =
      std::numeric_limits<float>::infinity();
  input.scene.push_back(MakeValidEmitter());

  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInvalidEnvironmentObservation));
}

TEST(EsrInputValidationTest, InvalidJammerSourceIsReported) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  session::EsrJammerSource jammer;
  jammer.center_hz = -1.0e9;       // 负频率
  jammer.bandwidth_hz = 0.0;       // 无效（非负但后续 ratio 也测）
  jammer.power_w = -10.0f;         // 负功率
  jammer.deception_risk = 2.0f;    // 超出 [0,1]
  jammer.confidence = -0.5f;       // 超出 [0,1]
  input.environment.jammer_sources.push_back(jammer);
  input.scene.push_back(MakeValidEmitter());

  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInvalidEnvironmentObservation));
  EXPECT_TRUE(HasValidationError(issues));
}

TEST(EsrInputValidationTest, NonFiniteJammerFieldIsReported) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  session::EsrJammerSource jammer;
  jammer.center_hz = 1.0e9;
  jammer.bandwidth_hz = 1.0e6;
  jammer.power_w = 10.0f;
  jammer.deception_risk = 0.5f;
  jammer.confidence = std::numeric_limits<float>::quiet_NaN();  // NaN
  input.environment.jammer_sources.push_back(jammer);
  input.scene.push_back(MakeValidEmitter());

  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInvalidEnvironmentObservation));
}

TEST(EsrInputValidationTest, ValidEnvironmentProducesNoError) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.environment.spectrum_occupancy_ratio = 0.3f;
  input.environment.atmospheric_observation.relative_humidity_ratio = 0.6f;
  input.environment.atmospheric_observation.precipitation_rate_mmph = 1.0f;
  input.environment.atmospheric_observation.visibility_km = 10.0f;
  session::EsrJammerSource jammer;
  jammer.center_hz = 1.0e9;
  jammer.bandwidth_hz = 1.0e6;
  jammer.power_w = 10.0f;
  jammer.deception_risk = 0.2f;
  jammer.confidence = 0.9f;
  input.environment.jammer_sources.push_back(jammer);
  input.scene.push_back(MakeValidEmitter());

  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  EXPECT_FALSE(HasValidationError(issues));
}

// =============================================================================
// 辐射源校验补充分支
// =============================================================================

TEST(EsrInputValidationTest, ZeroEmitterIdDoesNotProduceValidationError) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  session::EsrSceneEmitter emitter = MakeValidEmitter();
  emitter.emitter_id = 0U;
  input.scene.push_back(emitter);

  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  EXPECT_FALSE(HasValidationError(issues));
}

TEST(EsrInputValidationTest, InvalidEmitterBandwidthIsReportedAsError) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  session::EsrSceneEmitter emitter = MakeValidEmitter();
  emitter.bandwidth_hz = -1.0e6;
  input.scene.push_back(emitter);

  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInvalidEmitterBandwidth));
}

TEST(EsrInputValidationTest, InvalidEmitterPowerIsReportedAsError) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  session::EsrSceneEmitter emitter = MakeValidEmitter();
  emitter.tx_power_w = 0.0;
  input.scene.push_back(emitter);

  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kInvalidEmitterPower));
}

TEST(EsrInputValidationTest, NonFiniteEmitterVelocityIsReported) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  session::EsrSceneEmitter emitter = MakeValidEmitter();
  emitter.pose.velocity_mps.z = std::numeric_limits<float>::quiet_NaN();
  input.scene.push_back(emitter);

  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kNonFiniteEmitterNumericField));
}

TEST(EsrInputValidationTest, NonFiniteEmitterBeamStateIsReported) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  session::EsrSceneEmitter emitter = MakeValidEmitter();
  emitter.beam_state.center_el_deg = std::numeric_limits<float>::infinity();
  input.scene.push_back(emitter);

  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kNonFiniteEmitterNumericField));
}

TEST(EsrInputValidationTest, PlatformVelocityNonFiniteIsReported) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.platform_pose.velocity_mps.x = std::numeric_limits<float>::quiet_NaN();
  input.scene.push_back(MakeValidEmitter());

  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kNonFinitePlatformNumericField));
}

TEST(EsrInputValidationTest, PlatformPositionNonFiniteIsReported) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.platform_pose.position_m.y = std::numeric_limits<float>::infinity();
  input.scene.push_back(MakeValidEmitter());

  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  EXPECT_TRUE(ContainsCode(issues, ValidationCode::kNonFinitePlatformNumericField));
}

TEST(EsrInputValidationTest, ValidInputProducesNoErrors) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  input.platform_pose.position_m.x = 1000.0f;
  input.platform_pose.position_m.y = 2000.0f;
  input.platform_pose.position_m.z = 3000.0f;
  input.platform_altitude_m = 5000.0f;
  input.scene.push_back(MakeValidEmitter());

  const ValidationIssueList issues = ValidateEsrCycleInput(input);
  EXPECT_FALSE(HasValidationError(issues));
}

}  // namespace
}  // namespace session

}  // namespace electronic_surveillance_radar
