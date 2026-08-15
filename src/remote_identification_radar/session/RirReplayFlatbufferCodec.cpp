/**
 * @file RirReplayFlatbufferCodec.cpp
 * @brief 远程识别雷达 replay FlatBuffers 编解码实现。
 *
 * 表结构镜像 AR airborne_radar_replay.fbs 识别相关表；编码为字节精确往返
 * （无浮点近似），Decode* 失败返回 false 并回填 error。
 */

#include "remote_identification_radar/session/RirReplayFlatbufferCodec.h"

#include <cstdint>
#include <string>
#include <vector>

#include "remote_identification_radar/session/generated/rir_replay_generated.h"

namespace remote_identification_radar {
namespace session {

namespace fb = oneq::replay::remote_identification_radar::fb;

namespace {

flatbuffers::Offset<fb::RirRecognitionResultV1> EncodeRecognitionResult(
    flatbuffers::FlatBufferBuilder* builder, const RirRecognitionResult& value) {
  return fb::CreateRirRecognitionResultV1(
      *builder, static_cast<int>(value.state), static_cast<int>(value.target_category),
      builder->CreateString(value.target_model), value.confidence, value.best_score,
      value.runner_up_score, value.feature_scores.rcs_similarity, value.feature_scores.rcs_quality,
      value.feature_scores.motion_similarity, value.feature_scores.motion_quality,
      value.feature_scores.polarization_similarity, value.feature_scores.polarization_quality,
      value.feature_scores.range_profile_similarity, value.feature_scores.range_profile_quality,
      value.valid_feature_mask, value.observation_count, value.accumulation_sec,
      builder->CreateString(value.database_version), value.source_cycle_index,
      value.source_batch_id);
}

bool DecodeRecognitionResult(const fb::RirRecognitionResultV1* value, RirRecognitionResult* out) {
  if (value == nullptr) {
    return false;
  }
  const int state_raw = value->state();
  const int category_raw = value->target_category();
  if (state_raw < static_cast<int>(RirRecognitionState::kDisabled) ||
      state_raw > static_cast<int>(RirRecognitionState::kStale) ||
      category_raw < static_cast<int>(RirRecognitionCategory::kBallistic) ||
      category_raw > static_cast<int>(RirRecognitionCategory::kUav)) {
    return false;
  }
  out->state = static_cast<RirRecognitionState>(state_raw);
  out->target_category = static_cast<RirRecognitionCategory>(category_raw);
  if (value->target_model() != nullptr) {
    out->target_model = value->target_model()->str();
  }
  out->confidence = value->confidence();
  out->best_score = value->best_score();
  out->runner_up_score = value->runner_up_score();
  out->feature_scores.rcs_similarity = value->rcs_similarity();
  out->feature_scores.rcs_quality = value->rcs_quality();
  out->feature_scores.motion_similarity = value->motion_similarity();
  out->feature_scores.motion_quality = value->motion_quality();
  out->feature_scores.polarization_similarity = value->polarization_similarity();
  out->feature_scores.polarization_quality = value->polarization_quality();
  out->feature_scores.range_profile_similarity = value->range_profile_similarity();
  out->feature_scores.range_profile_quality = value->range_profile_quality();
  out->valid_feature_mask = value->valid_feature_mask();
  out->observation_count = value->observation_count();
  out->accumulation_sec = value->accumulation_sec();
  if (value->database_version() != nullptr) {
    out->database_version = value->database_version()->str();
  }
  out->source_cycle_index = value->source_cycle_index();
  out->source_batch_id = value->source_batch_id();
  return true;
}

flatbuffers::Offset<fb::RirRecognitionCycleSummaryV1> EncodeSummary(
    flatbuffers::FlatBufferBuilder* builder, const RirRecognitionCycleSummary& value) {
  return fb::CreateRirRecognitionCycleSummaryV1(
      *builder, value.participating_track_count, value.category_confirmed_count,
      value.model_confirmed_count, value.unknown_count, value.disabled_count,
      value.rcs_availability_rate, value.motion_availability_rate,
      value.polarization_availability_rate, value.range_profile_availability_rate,
      value.mean_confidence, value.mean_first_confirmation_sec, value.has_ground_truth,
      value.category_accuracy, value.model_accuracy);
}

void DecodeSummary(const fb::RirRecognitionCycleSummaryV1* value, RirRecognitionCycleSummary* out) {
  if (value == nullptr) {
    return;
  }
  out->participating_track_count = value->participating_track_count();
  out->category_confirmed_count = value->category_confirmed_count();
  out->model_confirmed_count = value->model_confirmed_count();
  out->unknown_count = value->unknown_count();
  out->disabled_count = value->disabled_count();
  out->rcs_availability_rate = value->rcs_availability_rate();
  out->motion_availability_rate = value->motion_availability_rate();
  out->polarization_availability_rate = value->polarization_availability_rate();
  out->range_profile_availability_rate = value->range_profile_availability_rate();
  out->mean_confidence = value->mean_confidence();
  out->mean_first_confirmation_sec = value->mean_first_confirmation_sec();
  out->has_ground_truth = value->has_ground_truth();
  out->category_accuracy = value->category_accuracy();
  out->model_accuracy = value->model_accuracy();
}

}  // namespace

std::string EncodeCycleReplayRecordFlatbuffer(const RirCycleReplayRecord& record) {
  flatbuffers::FlatBufferBuilder builder(1024);

  std::vector<flatbuffers::Offset<fb::RirTrackRecognitionOutputV1>> outputs;
  outputs.reserve(record.result.output_frame.recognition_outputs.size());
  for (std::size_t i = 0U; i < record.result.output_frame.recognition_outputs.size(); ++i) {
    const RirTrackRecognitionOutput& output = record.result.output_frame.recognition_outputs[i];
    outputs.push_back(fb::CreateRirTrackRecognitionOutputV1(
        builder, output.association_key, EncodeRecognitionResult(&builder, output.result)));
  }
  const flatbuffers::Offset<fb::RirOutputFrameV1> output_frame =
      fb::CreateRirOutputFrameV1(builder, record.result.output_frame.input_cycle_index,
                                 record.result.output_frame.batch_id,
                                 builder.CreateVector(outputs));

  flatbuffers::Offset<fb::RirRecognitionCycleSummaryV1> summary_offset;
  if (record.result.has_recognition_summary) {
    summary_offset = EncodeSummary(&builder, record.result.recognition_summary);
  }

  const flatbuffers::Offset<fb::RirCycleResultV1> result = fb::CreateRirCycleResultV1(
      builder, record.result.input_cycle_index, static_cast<int>(record.result.status),
      static_cast<int>(record.result.abort_reason), output_frame,
      record.result.has_recognition_summary, summary_offset);

  const flatbuffers::Offset<fb::RirSessionReplayStateV1> session_state =
      fb::CreateRirSessionReplayStateV1(
          builder, builder.CreateString(record.session_state.active_database_version));

  const flatbuffers::Offset<fb::RirCycleReplayRecordV1> root =
      fb::CreateRirCycleReplayRecordV1(builder, result, session_state);
  builder.Finish(root);
  return std::string(reinterpret_cast<const char*>(builder.GetBufferPointer()),
                     builder.GetSize());
}

bool DecodeCycleReplayRecordFlatbuffer(const std::string& payload_bytes,
                                       RirCycleReplayRecord* record, std::string* error) {
  if (record == nullptr) {
    if (error != nullptr) {
      *error = "record output must be non-null";
    }
    return false;
  }
  flatbuffers::Verifier verifier(reinterpret_cast<const std::uint8_t*>(payload_bytes.data()),
                                 payload_bytes.size());
  if (!fb::VerifyRirCycleReplayRecordV1Buffer(verifier)) {
    if (error != nullptr) {
      *error = "flatbuffer verification failed";
    }
    return false;
  }
  const fb::RirCycleReplayRecordV1* root = fb::GetRirCycleReplayRecordV1(payload_bytes.data());
  if (root == nullptr) {
    if (error != nullptr) {
      *error = "root table missing";
    }
    return false;
  }
  RirCycleReplayRecord candidate;
  const fb::RirCycleResultV1* result = root->result();
  if (result == nullptr) {
    if (error != nullptr) {
      *error = "result table missing";
    }
    return false;
  }
  candidate.result.input_cycle_index = result->input_cycle_index();
  const int status_raw = result->status();
  if (status_raw < static_cast<int>(RirCycleStatus::kCompleted) ||
      status_raw > static_cast<int>(RirCycleStatus::kRejectedExecution)) {
    if (error != nullptr) {
      *error = "invalid cycle status";
    }
    return false;
  }
  candidate.result.status = static_cast<RirCycleStatus>(status_raw);
  candidate.result.abort_reason = static_cast<RirCycleAbortReason>(result->abort_reason());
  candidate.result.has_recognition_summary = result->has_recognition_summary();
  DecodeSummary(result->recognition_summary(), &candidate.result.recognition_summary);

  const fb::RirOutputFrameV1* output_frame = result->output_frame();
  if (output_frame != nullptr) {
    candidate.result.output_frame.input_cycle_index = output_frame->input_cycle_index();
    candidate.result.output_frame.batch_id = output_frame->batch_id();
    const auto* outputs = output_frame->recognition_outputs();
    if (outputs != nullptr) {
      candidate.result.output_frame.recognition_outputs.reserve(outputs->size());
      for (const fb::RirTrackRecognitionOutputV1* output : *outputs) {
        if (output == nullptr) {
          continue;
        }
        RirTrackRecognitionOutput decoded;
        decoded.association_key = output->association_key();
        if (!DecodeRecognitionResult(output->result(), &decoded.result)) {
          if (error != nullptr) {
            *error = "invalid recognition result";
          }
          return false;
        }
        candidate.result.output_frame.recognition_outputs.push_back(decoded);
      }
    }
  }

  const fb::RirSessionReplayStateV1* session_state = root->session_state();
  if (session_state != nullptr && session_state->active_database_version() != nullptr) {
    candidate.session_state.active_database_version = session_state->active_database_version()->str();
  }

  *record = std::move(candidate);
  return true;
}

}  // namespace session
}  // namespace remote_identification_radar
