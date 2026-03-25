/**
 * @file esr_input_validation_test.cpp
 * @brief 验证 ESR 输入契约与基础校验规则。
 */

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

#include "1q/electronic_surveillance_radar/common/EmitterHypothesis.h"
#include "1q/electronic_surveillance_radar/common/EmitterTruthState.h"
#include "1q/electronic_surveillance_radar/core/context/EsrCycleInput.h"
#include "1q/electronic_surveillance_radar/core/context/EsrInputValidation.h"

namespace electronic_surveillance_radar {
namespace core {
namespace context {
namespace {

/**
 * @brief 检查校验结果列表中是否包含指定编码。
 * @param[in] issues 校验结果列表。
 * @param[in] code 目标编码。
 * @return 若包含指定编码则返回 `true`。
 */
bool ContainsCode(const EsrValidationIssueList& issues, EsrValidationCode code) {
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
common::EmitterTruthState MakeValidEmitter() {
  common::EmitterTruthState emitter;
  emitter.emitter_id = "E-1";
  emitter.carrier_hz = 10.0e9;
  emitter.bandwidth_hz = 2.0e6;
  emitter.tx_power_w = 1.0e6;
  emitter.pulse_width_s = 2.0e-6;
  emitter.pri_s = 2.0e-4;
  emitter.pose.position_m.x = 1000.0f;
  return emitter;
}

template <typename T, typename = void>
struct HasTruthEmitterId : std::false_type {};

template <typename T>
struct HasTruthEmitterId<T, decltype((void)std::declval<T>().truth_emitter_id)> : std::true_type {};

static_assert(!HasTruthEmitterId<common::EmitterHypothesis>::value,
              "EmitterHypothesis must not expose truth emitter id");

TEST(EsrInputValidationTest, InvalidCycleDeltaTimeIsReportedAsError) {
  EsrCycleInput input;
  input.dt_sec = 0.0f;
  input.scene_emitters.push_back(MakeValidEmitter());

  const EsrValidationIssueList issues = ValidateEsrCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, EsrValidationCode::kInvalidCycleDeltaTime));
  EXPECT_TRUE(HasEsrValidationError(issues));
}

TEST(EsrInputValidationTest, InvalidEmitterFrequencyIsReportedAsError) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  common::EmitterTruthState emitter = MakeValidEmitter();
  emitter.carrier_hz = 0.0;
  input.scene_emitters.push_back(emitter);

  const EsrValidationIssueList issues = ValidateEsrCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, EsrValidationCode::kInvalidEmitterFrequency));
  EXPECT_TRUE(HasEsrValidationError(issues));
}

TEST(EsrInputValidationTest, EmptySceneInputIsAllowed) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;

  const EsrValidationIssueList issues = ValidateEsrCycleInput(input);

  EXPECT_FALSE(HasEsrValidationError(issues));
  EXPECT_TRUE(issues.empty());
}

TEST(EsrInputValidationTest, NonFiniteEmitterNumericFieldIsReported) {
  EsrCycleInput input;
  input.dt_sec = 1.0f;
  common::EmitterTruthState emitter = MakeValidEmitter();
  emitter.bandwidth_hz = std::numeric_limits<double>::infinity();
  input.scene_emitters.push_back(emitter);

  const EsrValidationIssueList issues = ValidateEsrCycleInput(input);

  EXPECT_TRUE(ContainsCode(issues, EsrValidationCode::kNonFiniteEmitterNumericField));
  EXPECT_TRUE(HasEsrValidationError(issues));
}

}  // namespace
}  // namespace context
}  // namespace core
}  // namespace electronic_surveillance_radar
