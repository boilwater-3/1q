// Copyright 2026. All Rights Reserved.
//
// @file rir_replay_codec_roundtrip_test.cpp
// @brief RIR replay 编解码字节精确往返测试。
//
// 蓝本：ar_replay_codec_roundtrip_test.cpp 的识别字段段（审计基线 96de367c）；
// replay 逐周期比较语义（浮点容差 1e-5f）由调用方承担，本测试验证编码往返
// 无字段丢失（字节精确 + 逐字段断言）。

#include <gtest/gtest.h>

#include <string>

#include "1q/remote_identification_radar/session/RirCycleResult.h"
#include "1q/remote_identification_radar/session/RirRecognitionResult.h"
#include "remote_identification_radar/session/RirReplayCycleRecord.h"
#include "remote_identification_radar/session/RirReplayFlatbufferCodec.h"
#include "remote_identification_radar/session/generated/rir_replay_generated.h"

namespace remote_identification_radar {
namespace tests {
namespace {

using session::RirCycleAbortReason;
using session::RirCycleReplayRecord;
using session::RirCycleResult;
using session::RirCycleStatus;
using session::RirDesignationRevertReason;
using session::RirRecognitionCategory;
using session::RirRecognitionState;
using session::RirSessionReplayState;
using session::RirTrackRecognitionOutput;

TEST(RirReplayCodecRoundtripTest, RecognitionFieldsRoundtripPreserved) {
  RirCycleReplayRecord record;

  RirTrackRecognitionOutput output;
  output.association_key = 7U;
  output.result.state = RirRecognitionState::kModelConfirmed;
  output.result.target_category = RirRecognitionCategory::kBallistic;
  output.result.target_model = "BALLISTIC_EXAMPLE_A";
  output.result.confidence = 0.91f;
  output.result.best_score = 0.92f;
  output.result.runner_up_score = 0.81f;
  output.result.feature_scores.rcs_similarity = 0.9f;
  output.result.feature_scores.rcs_quality = 0.8f;
  output.result.feature_scores.motion_similarity = 0.7f;
  output.result.feature_scores.motion_quality = 0.6f;
  output.result.feature_scores.polarization_similarity = 0.5f;
  output.result.feature_scores.polarization_quality = 0.4f;
  output.result.feature_scores.range_profile_similarity = 0.3f;
  output.result.feature_scores.range_profile_quality = 0.2f;
  output.result.valid_feature_mask = 0x0BU;
  output.result.observation_count = 6U;
  output.result.accumulation_sec = 4.5f;
  output.result.database_version = "1.0.0";
  output.result.source_cycle_index = 12U;
  output.result.source_batch_id = 34U;
  record.result.output_frame.recognition_outputs.push_back(output);

  record.result.input_cycle_index = 12U;
  record.result.status = RirCycleStatus::kCompleted;
  record.result.abort_reason = RirCycleAbortReason::kNone;
  record.result.has_recognition_summary = true;
  record.result.recognition_summary.participating_track_count = 2U;
  record.result.recognition_summary.model_confirmed_count = 1U;
  record.result.recognition_summary.rcs_availability_rate = 0.5f;
  record.result.recognition_summary.mean_confidence = 0.91f;
  record.result.recognition_summary.has_ground_truth = true;
  record.result.recognition_summary.category_accuracy = 1.0f;
  record.result.recognition_summary.model_accuracy = 1.0f;
  record.session_state.active_database_version = "1.0.0";
  record.session_state.detection_random_seed = 123U;
  record.result.recognition_summary.dwell_budget.scheduled_dwell_count = 2U;
  record.result.recognition_summary.dwell_budget.executed_dwell_count = 1U;
  record.result.recognition_summary.dwell_budget.dwell_budget_sec = 0.1f;
  record.result.recognition_summary.dwell_budget.dwell_consumed_sec = 0.05f;
  record.result.designated_target_id = 9100U;
  record.result.designation_active = true;
  record.result.designation_reverted_to_scan = false;
  record.result.designation_revert_reason = RirDesignationRevertReason::kNone;
  record.result.dwell_center_deg.az_deg = -56.0f;
  record.result.dwell_center_deg.el_deg = 30.0f;

  const std::string encoded = session::EncodeCycleReplayRecordFlatbuffer(record);
  ASSERT_FALSE(encoded.empty());
  RirCycleReplayRecord decoded;
  std::string error;
  ASSERT_TRUE(session::DecodeCycleReplayRecordFlatbuffer(encoded, &decoded, &error)) << error;
  // 字节精确往返：任何字段丢失都会 divergence。
  EXPECT_EQ(session::EncodeCycleReplayRecordFlatbuffer(decoded), encoded);

  EXPECT_EQ(decoded.result.input_cycle_index, 12U);
  EXPECT_EQ(decoded.result.status, RirCycleStatus::kCompleted);
  EXPECT_EQ(decoded.result.abort_reason, RirCycleAbortReason::kNone);
  EXPECT_TRUE(decoded.result.has_recognition_summary);
  EXPECT_EQ(decoded.result.recognition_summary.participating_track_count, 2U);
  EXPECT_EQ(decoded.result.recognition_summary.model_confirmed_count, 1U);
  EXPECT_FLOAT_EQ(decoded.result.recognition_summary.rcs_availability_rate, 0.5f);
  EXPECT_FLOAT_EQ(decoded.result.recognition_summary.mean_confidence, 0.91f);
  EXPECT_TRUE(decoded.result.recognition_summary.has_ground_truth);
  EXPECT_FLOAT_EQ(decoded.result.recognition_summary.category_accuracy, 1.0f);
  EXPECT_FLOAT_EQ(decoded.result.recognition_summary.model_accuracy, 1.0f);

  ASSERT_EQ(decoded.result.output_frame.recognition_outputs.size(), 1U);
  const auto& decoded_output = decoded.result.output_frame.recognition_outputs.front();
  EXPECT_EQ(decoded_output.association_key, 7U);
  EXPECT_EQ(decoded_output.result.state, RirRecognitionState::kModelConfirmed);
  EXPECT_EQ(decoded_output.result.target_category, RirRecognitionCategory::kBallistic);
  EXPECT_EQ(decoded_output.result.target_model, "BALLISTIC_EXAMPLE_A");
  EXPECT_FLOAT_EQ(decoded_output.result.confidence, 0.91f);
  EXPECT_FLOAT_EQ(decoded_output.result.best_score, 0.92f);
  EXPECT_FLOAT_EQ(decoded_output.result.runner_up_score, 0.81f);
  EXPECT_FLOAT_EQ(decoded_output.result.feature_scores.rcs_similarity, 0.9f);
  EXPECT_FLOAT_EQ(decoded_output.result.feature_scores.rcs_quality, 0.8f);
  EXPECT_FLOAT_EQ(decoded_output.result.feature_scores.motion_similarity, 0.7f);
  EXPECT_FLOAT_EQ(decoded_output.result.feature_scores.motion_quality, 0.6f);
  EXPECT_FLOAT_EQ(decoded_output.result.feature_scores.polarization_similarity, 0.5f);
  EXPECT_FLOAT_EQ(decoded_output.result.feature_scores.polarization_quality, 0.4f);
  EXPECT_FLOAT_EQ(decoded_output.result.feature_scores.range_profile_similarity, 0.3f);
  EXPECT_FLOAT_EQ(decoded_output.result.feature_scores.range_profile_quality, 0.2f);
  EXPECT_EQ(decoded_output.result.valid_feature_mask, 0x0BU);
  EXPECT_EQ(decoded_output.result.observation_count, 6U);
  EXPECT_FLOAT_EQ(decoded_output.result.accumulation_sec, 4.5f);
  EXPECT_EQ(decoded_output.result.database_version, "1.0.0");
  EXPECT_EQ(decoded_output.result.source_cycle_index, 12U);
  EXPECT_EQ(decoded_output.result.source_batch_id, 34U);
  EXPECT_EQ(decoded.session_state.active_database_version, "1.0.0");
  EXPECT_EQ(decoded.session_state.detection_random_seed, 123U);
  EXPECT_EQ(decoded.result.recognition_summary.dwell_budget.scheduled_dwell_count, 2U);
  EXPECT_EQ(decoded.result.recognition_summary.dwell_budget.executed_dwell_count, 1U);
  EXPECT_FLOAT_EQ(decoded.result.recognition_summary.dwell_budget.dwell_budget_sec, 0.1f);
  EXPECT_FLOAT_EQ(decoded.result.recognition_summary.dwell_budget.dwell_consumed_sec, 0.05f);
  EXPECT_EQ(decoded.result.designated_target_id, 9100U);
  EXPECT_TRUE(decoded.result.designation_active);
  EXPECT_FALSE(decoded.result.designation_reverted_to_scan);
  EXPECT_EQ(decoded.result.designation_revert_reason, RirDesignationRevertReason::kNone);
  EXPECT_FLOAT_EQ(decoded.result.dwell_center_deg.az_deg, -56.0f);
  EXPECT_FLOAT_EQ(decoded.result.dwell_center_deg.el_deg, 30.0f);
}

TEST(RirReplayCodecRoundtripTest, DefaultStateRoundtripsByteExact) {
  RirCycleReplayRecord record;
  const std::string encoded = session::EncodeCycleReplayRecordFlatbuffer(record);
  ASSERT_FALSE(encoded.empty());
  RirCycleReplayRecord decoded;
  std::string error;
  ASSERT_TRUE(session::DecodeCycleReplayRecordFlatbuffer(encoded, &decoded, &error)) << error;
  EXPECT_EQ(session::EncodeCycleReplayRecordFlatbuffer(decoded), encoded);
  EXPECT_EQ(decoded.result.status, RirCycleStatus::kRejectedInvalidInput);
  EXPECT_FALSE(decoded.result.has_recognition_summary);
  EXPECT_TRUE(decoded.result.output_frame.recognition_outputs.empty());
  EXPECT_TRUE(decoded.session_state.active_database_version.empty());
}

TEST(RirReplayCodecRoundtripTest, RejectsPayloadWithoutV2Identifier) {
  RirCycleReplayRecord record;
  record.session_state.detection_random_seed = 7U;
  std::string encoded = session::EncodeCycleReplayRecordFlatbuffer(record);
  ASSERT_GE(encoded.size(), 8U);
  // 抹掉 file_identifier "RIR2"，模拟阶段 1 V1/旧记录：必须显式拒绝而非静默误读。
  const char* identifier = flatbuffers::GetBufferIdentifier(encoded.data());
  encoded[static_cast<std::size_t>(identifier - encoded.data())] = 'X';
  encoded[static_cast<std::size_t>(identifier - encoded.data()) + 1U] = 'X';
  encoded[static_cast<std::size_t>(identifier - encoded.data()) + 2U] = 'X';
  encoded[static_cast<std::size_t>(identifier - encoded.data()) + 3U] = 'X';
  RirCycleReplayRecord decoded;
  std::string error;
  EXPECT_FALSE(session::DecodeCycleReplayRecordFlatbuffer(encoded, &decoded, &error));
  EXPECT_FALSE(error.empty());
}

}  // namespace
}  // namespace tests
}  // namespace remote_identification_radar
