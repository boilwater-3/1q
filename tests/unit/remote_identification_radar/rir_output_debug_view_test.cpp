// Copyright 2026. All Rights Reserved.
//
// @file rir_output_debug_view_test.cpp
// @brief 目标输出调试视图构建器单元测试（观测投影 DebugView，规则 12）。
//
// 直接构造 RirCycleInput + RirCycleResult 调用 Build()，覆盖状态表
// （确认/候选/丢失/无航迹回填/零 ID）、非完成周期、指定任务镜像转写、
// 识别诊断映射与问题列表转写。

#include "1q/remote_identification_radar/session/RirOutputDebugView.h"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "1q/remote_identification_radar/session/RirCycleInput.h"
#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirIssueCodes.h"
#include "1q/foundation/validation_types.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using session::RirCycleInput;
using session::RirCycleResult;
using session::RirCycleStatus;
using session::RirDebugTargetStatus;
using session::RirOutputDebugViewBuilder;
using session::RirSceneTarget;
using session::RirTrackAttributionRecord;
using session::RirTrackLifecycleStatus;

RirSceneTarget MakeTarget(std::uint64_t id, const char* name, float position_x) {
  RirSceneTarget target;
  target.external_target_id = id;
  target.target_name = name;
  target.position_x = position_x;
  target.position_z = 2000.0f;
  return target;
}

RirTrackAttributionRecord MakeAttribution(std::uint64_t key, std::uint64_t id,
                                          RirTrackLifecycleStatus status) {
  session::RirTrackAttributionRecord attribution;
  attribution.association_key = key;
  attribution.external_target_id = id;
  attribution.target_name = "truth-" + std::to_string(id);
  attribution.track_status = status;
  attribution.hit_count = 3U;
  attribution.position_enu_x_m = 100.0 + static_cast<double>(id);
  attribution.position_enu_y_m = 50.0;
  attribution.position_enu_z_m = 2000.0;
  attribution.speed_m_per_s = 42.0;
  return attribution;
}

RirCycleResult MakeCompletedResult(std::uint32_t cycle_index,
                                   std::vector<RirTrackAttributionRecord> attributions) {
  RirCycleResult result;
  result.input_cycle_index = cycle_index;
  result.status = RirCycleStatus::kCompleted;
  result.track_attributions = std::move(attributions);
  return result;
}

const session::RirDebugTargetState* FindState(const session::RirOutputDebugView& view,
                                              std::uint64_t id) {
  for (const session::RirDebugTargetState& state : view.targets) {
    if (state.external_target_id == id) {
      return &state;
    }
  }
  return nullptr;
}

/// @brief 状态表：归属状态逐值映射；无航迹目标 kNotInOutput 且带输入几何回填。
TEST(RirOutputDebugViewTest, BuildMapsAttributionStatusToDebugStates) {
  RirCycleInput input;
  input.input_cycle_index = 5U;
  input.scene_targets = {MakeTarget(1001U, "alpha", 3000.0f),
                         MakeTarget(1002U, "beta", 6000.0f), MakeTarget(1003U, "gamma", 9000.0f)};

  std::vector<RirTrackAttributionRecord> attributions;
  attributions.push_back(MakeAttribution(10U, 1001U, RirTrackLifecycleStatus::kConfirmed));
  attributions.push_back(MakeAttribution(20U, 1002U, RirTrackLifecycleStatus::kTentative));
  attributions.push_back(MakeAttribution(30U, 1003U, RirTrackLifecycleStatus::kLost));

  const session::RirOutputDebugView view =
      RirOutputDebugViewBuilder::Build(input, MakeCompletedResult(5U, attributions));

  EXPECT_EQ(view.input_cycle_index, 5U);
  EXPECT_TRUE(view.executed_this_cycle);
  ASSERT_EQ(view.targets.size(), 3U);

  const session::RirDebugTargetState* confirmed = FindState(view, 1001U);
  ASSERT_NE(confirmed, nullptr);
  EXPECT_EQ(confirmed->status, RirDebugTargetStatus::kConfirmed);
  EXPECT_TRUE(confirmed->has_track);
  EXPECT_EQ(confirmed->association_key, 10U);
  EXPECT_EQ(confirmed->hit_count, 3U);
  EXPECT_DOUBLE_EQ(confirmed->position_enu_x_m, 1101.0);
  EXPECT_DOUBLE_EQ(confirmed->speed_m_per_s, 42.0);

  EXPECT_EQ(FindState(view, 1002U)->status, RirDebugTargetStatus::kTentative);
  EXPECT_EQ(FindState(view, 1003U)->status, RirDebugTargetStatus::kLost);
}

/// @brief 无航迹目标：kNotInOutput + 输入几何回填（斜距/视线角，规则 12）。
TEST(RirOutputDebugViewTest, UntrackedTargetBackfillsInputGeometry) {
  RirCycleInput input;
  input.input_cycle_index = 1U;
  input.scene_targets = {MakeTarget(1001U, "alpha", 3000.0f)};

  const session::RirOutputDebugView view =
      RirOutputDebugViewBuilder::Build(input, MakeCompletedResult(1U, {}));

  ASSERT_EQ(view.targets.size(), 1U);
  const session::RirDebugTargetState& state = view.targets[0];
  EXPECT_EQ(state.status, RirDebugTargetStatus::kNotInOutput);
  EXPECT_FALSE(state.has_track);
  EXPECT_EQ(state.association_key, 0U);
  // 输入几何回填：斜距 = √(3000² + 2000²)，az=0°（正东），el=atan2(2000, 3000)。
  // 斜距现由 ENU 位置双精度推导：float 字面量舍入差 ~6e-6 @3600 m，容差 1e-4。
  EXPECT_NEAR(state.slant_range_m, std::sqrt(3000.0f * 3000.0f + 2000.0f * 2000.0f), 1e-4);
  EXPECT_NEAR(state.look_az_deg, 0.0, 1e-6);
  EXPECT_NEAR(state.look_el_deg, 57.29577951308232 * std::atan2(2000.0, 3000.0), 1e-6);
  EXPECT_FALSE(state.has_recognition_output);
}

/// @brief 零 ID 目标：无法按 ID 关联 → kNotInOutput（对齐 AR 行为）。
TEST(RirOutputDebugViewTest, ZeroIdTargetIsNotInOutput) {
  RirCycleInput input;
  input.input_cycle_index = 1U;
  input.scene_targets = {MakeTarget(0U, "anon", 3000.0f)};

  const session::RirOutputDebugView view =
      RirOutputDebugViewBuilder::Build(input, MakeCompletedResult(1U, {}));

  ASSERT_EQ(view.targets.size(), 1U);
  EXPECT_EQ(view.targets[0].status, RirDebugTargetStatus::kNotInOutput);
}

/// @brief 非完成周期：全部目标 kCycleNotCompleted，周期头携带中止原因与 issues 转写。
TEST(RirOutputDebugViewTest, NonCompletedCycleMarksAllTargets) {
  RirCycleInput input;
  input.input_cycle_index = 7U;
  input.scene_targets = {MakeTarget(1001U, "alpha", 3000.0f)};

  RirCycleResult result;
  result.input_cycle_index = 7U;
  result.status = RirCycleStatus::kPoweredOff;
  result.abort_reason = session::RirCycleAbortReason::kPoweredOff;
  session::RirIssue issue;
  issue.code = session::codes::kTargetModeNotIdentify;
  issue.severity = session::RirIssueSeverity::kInfo;
  result.issues.push_back(issue);

  const session::RirOutputDebugView view = RirOutputDebugViewBuilder::Build(input, result);

  EXPECT_FALSE(view.executed_this_cycle);
  EXPECT_EQ(view.abort_reason, session::RirCycleAbortReason::kPoweredOff);
  ASSERT_EQ(view.targets.size(), 1U);
  EXPECT_EQ(view.targets[0].status, RirDebugTargetStatus::kCycleNotCompleted);
  EXPECT_FALSE(view.targets[0].has_track);
  ASSERT_EQ(view.issues.size(), 1U);
  EXPECT_EQ(view.issues[0].code, "rir.target_mode_not_identify");
}

/// @brief 指定任务镜像：信封五字段逐字转写。
TEST(RirOutputDebugViewTest, BuildTranscribesDesignationMirror) {
  RirCycleInput input;
  input.input_cycle_index = 9U;
  input.scene_targets = {MakeTarget(1001U, "alpha", 3000.0f)};

  RirCycleResult result = MakeCompletedResult(9U, {});
  result.designated_target_id = 1001U;
  result.designation_active = true;
  result.designation_reverted_to_scan = false;
  result.designation_revert_reason = session::RirDesignationRevertReason::kNone;
  result.dwell_center_deg.az_deg = 12.5f;
  result.dwell_center_deg.el_deg = 3.25f;

  const session::RirOutputDebugView view = RirOutputDebugViewBuilder::Build(input, result);

  EXPECT_EQ(view.designated_target_id, 1001U);
  EXPECT_TRUE(view.designation_active);
  EXPECT_FALSE(view.designation_reverted_to_scan);
  EXPECT_FLOAT_EQ(view.dwell_center_deg.az_deg, 12.5f);
  EXPECT_FLOAT_EQ(view.dwell_center_deg.el_deg, 3.25f);
}

/// @brief 识别诊断：出口②有非 kDisabled 结论时透出；无结论时保持默认。
TEST(RirOutputDebugViewTest, RecognitionDiagnosticsMirrorOutletTwo) {
  RirCycleInput input;
  input.input_cycle_index = 4U;
  input.scene_targets = {MakeTarget(1001U, "alpha", 3000.0f),
                         MakeTarget(1002U, "beta", 6000.0f)};

  std::vector<RirTrackAttributionRecord> attributions;
  attributions.push_back(MakeAttribution(10U, 1001U, RirTrackLifecycleStatus::kConfirmed));
  attributions.push_back(MakeAttribution(20U, 1002U, RirTrackLifecycleStatus::kTentative));

  RirCycleResult result = MakeCompletedResult(4U, attributions);

  session::RirTrackRecognitionOutput recognized;
  recognized.association_key = 10U;
  recognized.result.state = session::RirRecognitionState::kModelConfirmed;
  recognized.result.target_category = session::RirRecognitionCategory::kFighter;
  recognized.result.target_model = "F-16C";
  recognized.result.confidence = 0.87f;
  recognized.result.observation_count = 12U;
  result.output_frame.recognition_outputs.push_back(recognized);

  // 目标 1002 的键无出口②条目（默认 kDisabled 语义不透出）。
  session::RirTrackRecognitionOutput absent;
  absent.association_key = 20U;
  absent.result.state = session::RirRecognitionState::kDisabled;
  result.output_frame.recognition_outputs.push_back(absent);

  const session::RirOutputDebugView view = RirOutputDebugViewBuilder::Build(input, result);

  const session::RirDebugTargetState* alpha = FindState(view, 1001U);
  ASSERT_NE(alpha, nullptr);
  EXPECT_TRUE(alpha->has_recognition_output);
  EXPECT_EQ(alpha->recognition_state, session::RirRecognitionState::kModelConfirmed);
  EXPECT_EQ(alpha->target_category, session::RirRecognitionCategory::kFighter);
  EXPECT_EQ(alpha->target_model, "F-16C");
  EXPECT_FLOAT_EQ(alpha->confidence, 0.87f);
  EXPECT_EQ(alpha->observation_count, 12U);

  const session::RirDebugTargetState* beta = FindState(view, 1002U);
  ASSERT_NE(beta, nullptr);
  EXPECT_FALSE(beta->has_recognition_output);
  EXPECT_EQ(beta->recognition_state, session::RirRecognitionState::kDisabled);
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
