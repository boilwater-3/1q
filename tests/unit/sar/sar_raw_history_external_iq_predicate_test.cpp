// 谓词守护：验证 HasExternalRawIq 以 IQ 样本为充要条件。
//
// 历史背景：该谓词曾是 6 个 raw_iq 字段的析取，导致仅填伴随轨迹（pulse_states/pulse_count）
// 的输入被误判为"外部完整 IQ"。收紧后，只有同时
// 提供 samples_per_pulse + i_values + q_values 才视为外部 IQ；仅轨迹不再触发外部
// 路径。本测试守护该契约，防回退。

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "1q/sar/session/SarCycleInput.h"
#include "sar/session/SarRawHistoryBuilder.h"

namespace sar {
namespace session {
namespace {

SarRawIqFrame::PulseState MakePulseState(std::uint64_t pulse_id) {
  SarRawIqFrame::PulseState state;
  state.pulse_id = pulse_id;
  state.time_s = static_cast<double>(pulse_id);
  state.position_x_m = static_cast<double>(pulse_id);
  state.velocity_x_mps = 1.0;
  return state;
}

TEST(SarRawHistoryExternalIqPredicateTest, EmptyInputIsNotExternal) {
  SarCycleInput input;
  EXPECT_FALSE(HasExternalRawIq(input));
}

TEST(SarRawHistoryExternalIqPredicateTest, TrajectoryOnlyIsNotExternal) {
  // 仅伴随轨迹（pulse_states），无 IQ 样本——不得判为外部 IQ。
  // 这是修复的核心行为：防回退到"析取误判"。
  SarCycleInput input;
  for (std::uint64_t i = 0U; i < 9U; ++i) {
    input.raw_iq.pulse_states.push_back(MakePulseState(i));
  }
  input.raw_iq.ideal_pulse_states = input.raw_iq.pulse_states;
  EXPECT_FALSE(HasExternalRawIq(input));
}

TEST(SarRawHistoryExternalIqPredicateTest, SamplesWithoutTrajectoryIsExternal) {
  // 有 IQ 样本即视为外部 IQ，无论是否附带轨迹。
  SarCycleInput input;
  input.raw_iq.samples_per_pulse = 64U;
  input.raw_iq.i_values.assign(9U * 64U, 0.0);
  input.raw_iq.q_values.assign(9U * 64U, 0.0);
  EXPECT_TRUE(HasExternalRawIq(input));
}

TEST(SarRawHistoryExternalIqPredicateTest, PartialSamplesIsNotExternal) {
  // IQ 样本是充要条件：缺 i_values 或 q_values 之一即不视为外部 IQ。
  SarCycleInput input;
  input.raw_iq.samples_per_pulse = 64U;
  input.raw_iq.i_values.assign(9U * 64U, 0.0);
  // q_values 缺失。
  EXPECT_FALSE(HasExternalRawIq(input));
}

}  // namespace
}  // namespace session
}  // namespace sar
