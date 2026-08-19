/**
 * @file RirReplayFlatbufferCodec.cpp
 * @brief 远程识别雷达 replay FlatBuffers 编解码实现。
 *
 * 阶段 2-S 起编码 V2 表；旧 V1 记录显式拒绝（破坏性版本，不静默误读）。
 * 编码为字节精确往返（无浮点近似），Decode* 失败返回 false 并回填 error。
 */

#include "remote_identification_radar/session/RirReplayFlatbufferCodec.h"

#include <cstdint>
#include <string>
#include <vector>

#include "1q/coordinate/types.h"
#include "1q/electromagnetics/RfScene.h"
#include "remote_identification_radar/session/generated/rir_replay_generated.h"

namespace remote_identification_radar {
namespace session {

namespace fb = oneq::replay::remote_identification_radar::fb;

namespace {

flatbuffers::Offset<fb::Vector3d> EncodeRfV2Position(flatbuffers::FlatBufferBuilder* builder,
                                                      const oneq::coordinate::EcefPositionM& value) {
  return fb::CreateVector3d(*builder, value.x_m, value.y_m, value.z_m);
}

flatbuffers::Offset<fb::Vector3d> EncodeRfV2Velocity(
    flatbuffers::FlatBufferBuilder* builder, const oneq::coordinate::EcefVelocityMps& value) {
  return fb::CreateVector3d(*builder, value.x_mps, value.y_mps, value.z_mps);
}

flatbuffers::Offset<fb::RfSceneDirectionV2> EncodeRfV2Direction(
    flatbuffers::FlatBufferBuilder* builder,
    const oneq::electromagnetics::RfSceneDirection& value) {
  return fb::CreateRfSceneDirectionV2(*builder, value.x, value.y, value.z);
}

flatbuffers::Offset<fb::RfSceneAntennaPatternV2> EncodeRfV2Antenna(
    flatbuffers::FlatBufferBuilder* builder,
    const oneq::electromagnetics::RfSceneAntennaPattern& value) {
  return fb::CreateRfSceneAntennaPatternV2(
      *builder, EncodeRfV2Direction(builder, value.boresight_ecef), value.peak_gain_dbi,
      value.half_power_beamwidth_deg, value.sidelobe_level_db, value.backlobe_level_db,
      value.cross_polarization_isolation_db);
}

flatbuffers::Offset<fb::RfEmissionIdentityV2> EncodeRfV2Identity(
    flatbuffers::FlatBufferBuilder* builder,
    const oneq::electromagnetics::RfEmissionIdentity& value) {
  return fb::CreateRfEmissionIdentityV2(*builder, value.platform_id, value.equipment_id,
                                        value.emission_id);
}

flatbuffers::Offset<fb::RfWaveformScheduleV2> EncodeRfV2Waveform(
    flatbuffers::FlatBufferBuilder* builder,
    const oneq::electromagnetics::RfWaveformSchedule& value) {
  return fb::CreateRfWaveformScheduleV2(
      *builder, static_cast<int>(value.kind), value.activity_start_time_s,
      value.activity_duration_s, value.center_frequency_hz, value.occupied_bandwidth_hz,
      value.transmit_power_w, value.pulse_width_s, value.pulse_repetition_interval_s,
      value.first_pulse_time_s, value.pulse_count, value.pulse_jitter_fraction, value.timing_seed,
      value.timing_epoch, value.sweep_start_frequency_hz, value.sweep_stop_frequency_hz,
      value.sweep_period_s);
}

flatbuffers::Offset<fb::RfSceneEmissionV2> EncodeRfV2Emission(
    flatbuffers::FlatBufferBuilder* builder, const oneq::electromagnetics::RfSceneEmission& value) {
  return fb::CreateRfSceneEmissionV2(*builder, EncodeRfV2Identity(builder, value.identity),
                                     EncodeRfV2Position(builder, value.position_ecef_m),
                                     EncodeRfV2Velocity(builder, value.velocity_ecef_mps),
                                     EncodeRfV2Antenna(builder, value.antenna),
                                     static_cast<int>(value.polarization),
                                     EncodeRfV2Waveform(builder, value.waveform));
}

flatbuffers::Offset<fb::RfSceneFrameV2> EncodeRfV2Scene(
    flatbuffers::FlatBufferBuilder* builder, const oneq::electromagnetics::RfSceneFrame& value) {
  std::vector<flatbuffers::Offset<fb::RfSceneEmissionV2>> emissions;
  emissions.reserve(value.emissions.size());
  for (const auto& emission : value.emissions) {
    emissions.push_back(EncodeRfV2Emission(builder, emission));
  }
  return fb::CreateRfSceneFrameV2(*builder, value.world_cycle_index, value.window_start_time_s,
                                  value.window_duration_s, builder->CreateVector(emissions));
}

oneq::coordinate::EcefPositionM DecodeRfV2Position(const fb::Vector3d* value) {
  oneq::coordinate::EcefPositionM result;
  if (value != nullptr) {
    result.x_m = value->x();
    result.y_m = value->y();
    result.z_m = value->z();
  }
  return result;
}

oneq::coordinate::EcefVelocityMps DecodeRfV2Velocity(const fb::Vector3d* value) {
  oneq::coordinate::EcefVelocityMps result;
  if (value != nullptr) {
    result.x_mps = value->x();
    result.y_mps = value->y();
    result.z_mps = value->z();
  }
  return result;
}

oneq::electromagnetics::RfSceneDirection DecodeRfV2Direction(const fb::RfSceneDirectionV2* value) {
  oneq::electromagnetics::RfSceneDirection result;
  if (value != nullptr) {
    result.x = value->x();
    result.y = value->y();
    result.z = value->z();
  }
  return result;
}

oneq::electromagnetics::RfSceneAntennaPattern DecodeRfV2Antenna(
    const fb::RfSceneAntennaPatternV2* value) {
  oneq::electromagnetics::RfSceneAntennaPattern result;
  if (value != nullptr) {
    result.boresight_ecef = DecodeRfV2Direction(value->boresight_ecef());
    result.peak_gain_dbi = value->peak_gain_dbi();
    result.half_power_beamwidth_deg = value->half_power_beamwidth_deg();
    result.sidelobe_level_db = value->sidelobe_level_db();
    result.backlobe_level_db = value->backlobe_level_db();
    result.cross_polarization_isolation_db = value->cross_polarization_isolation_db();
  }
  return result;
}

oneq::electromagnetics::RfWaveformSchedule DecodeRfV2Waveform(
    const fb::RfWaveformScheduleV2* value) {
  oneq::electromagnetics::RfWaveformSchedule result;
  if (value != nullptr) {
    result.kind = static_cast<oneq::electromagnetics::RfSceneWaveformKind>(value->kind());
    result.activity_start_time_s = value->activity_start_time_s();
    result.activity_duration_s = value->activity_duration_s();
    result.center_frequency_hz = value->center_frequency_hz();
    result.occupied_bandwidth_hz = value->occupied_bandwidth_hz();
    result.transmit_power_w = value->transmit_power_w();
    result.pulse_width_s = value->pulse_width_s();
    result.pulse_repetition_interval_s = value->pulse_repetition_interval_s();
    result.first_pulse_time_s = value->first_pulse_time_s();
    result.pulse_count = value->pulse_count();
    result.pulse_jitter_fraction = value->pulse_jitter_fraction();
    result.timing_seed = value->timing_seed();
    result.timing_epoch = value->timing_epoch();
    result.sweep_start_frequency_hz = value->sweep_start_frequency_hz();
    result.sweep_stop_frequency_hz = value->sweep_stop_frequency_hz();
    result.sweep_period_s = value->sweep_period_s();
  }
  return result;
}

oneq::electromagnetics::RfSceneEmission DecodeRfV2Emission(const fb::RfSceneEmissionV2* value) {
  oneq::electromagnetics::RfSceneEmission result;
  if (value != nullptr) {
    if (value->identity() != nullptr) {
      result.identity.platform_id = value->identity()->platform_id();
      result.identity.equipment_id = value->identity()->equipment_id();
      result.identity.emission_id = value->identity()->emission_id();
    }
    result.position_ecef_m = DecodeRfV2Position(value->position_ecef_m());
    result.velocity_ecef_mps = DecodeRfV2Velocity(value->velocity_ecef_mps());
    result.antenna = DecodeRfV2Antenna(value->antenna());
    result.polarization =
        static_cast<oneq::electromagnetics::RfScenePolarization>(value->polarization());
    result.waveform = DecodeRfV2Waveform(value->waveform());
  }
  return result;
}

oneq::electromagnetics::RfSceneFrame DecodeRfV2Scene(const fb::RfSceneFrameV2* value) {
  oneq::electromagnetics::RfSceneFrame result;
  if (value != nullptr) {
    result.world_cycle_index = value->world_cycle_index();
    result.window_start_time_s = value->window_start_time_s();
    result.window_duration_s = value->window_duration_s();
    if (value->emissions() != nullptr) {
      result.emissions.reserve(value->emissions()->size());
      for (const fb::RfSceneEmissionV2* emission : *value->emissions()) {
        result.emissions.push_back(DecodeRfV2Emission(emission));
      }
    }
  }
  return result;
}

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

flatbuffers::Offset<fb::RirFeatureMeasurementRecordV1> EncodeFeatureMeasurement(
    flatbuffers::FlatBufferBuilder* builder, const RirFeatureMeasurementRecord& value) {
  const flatbuffers::Offset<fb::RirRcsFeatureObservationV1> rcs =
      fb::CreateRirRcsFeatureObservationV1(
          *builder, value.features.rcs.valid, value.features.rcs.mean_dbsm,
          value.features.rcs.std_db, value.features.rcs.azimuth_variation_db,
          value.features.rcs.elevation_variation_db, value.features.rcs.peak_to_valley_db,
          value.features.rcs.aspect_coverage_deg, value.features.rcs.quality);
  const flatbuffers::Offset<fb::RirMotionFeatureObservationV1> motion =
      fb::CreateRirMotionFeatureObservationV1(
          *builder, value.features.motion.valid, value.features.motion.speed_m_per_s,
          value.features.motion.altitude_m, value.features.motion.acceleration_m_per_s2,
          value.features.motion.turn_radius_m, value.features.motion.is_straight,
          value.features.motion.quality);
  const flatbuffers::Offset<fb::RirPolarizationFeatureObservationV1> polarization =
      fb::CreateRirPolarizationFeatureObservationV1(
          *builder, value.features.polarization.valid,
          value.features.polarization.energy_difference_db,
          value.features.polarization.relative_difference_db,
          value.features.polarization.energy_sum_db, value.features.polarization.quality);
  const flatbuffers::Offset<fb::RirRangeProfileFeatureObservationV1> range_profile =
      fb::CreateRirRangeProfileFeatureObservationV1(
          *builder, value.features.range_profile.valid, value.features.range_profile.length_m,
          value.features.range_profile.peak_count,
          value.features.range_profile.peak_energy_concentration,
          value.features.range_profile.resolution_m, value.features.range_profile.quality);
  return fb::CreateRirFeatureMeasurementRecordV1(
      *builder, value.association_key, rcs, motion, polarization, range_profile,
      value.valid_feature_mask, value.look_az_deg, value.look_el_deg, value.range_m,
      value.snr_db, value.dwell_sec, value.bandwidth_hz, value.has_platform_position,
      value.platform_position.x_m, value.platform_position.y_m, value.platform_position.z_m,
      value.cycle_index, value.batch_id);
}

void DecodeFeatureMeasurement(const fb::RirFeatureMeasurementRecordV1* value,
                               RirFeatureMeasurementRecord* out) {
  if (value == nullptr) {
    return;
  }
  out->association_key = value->association_key();
  if (value->rcs() != nullptr) {
    out->features.rcs.valid = value->rcs()->valid();
    out->features.rcs.mean_dbsm = value->rcs()->mean_dbsm();
    out->features.rcs.std_db = value->rcs()->std_db();
    out->features.rcs.azimuth_variation_db = value->rcs()->azimuth_variation_db();
    out->features.rcs.elevation_variation_db = value->rcs()->elevation_variation_db();
    out->features.rcs.peak_to_valley_db = value->rcs()->peak_to_valley_db();
    out->features.rcs.aspect_coverage_deg = value->rcs()->aspect_coverage_deg();
    out->features.rcs.quality = value->rcs()->quality();
  }
  if (value->motion() != nullptr) {
    out->features.motion.valid = value->motion()->valid();
    out->features.motion.speed_m_per_s = value->motion()->speed_m_per_s();
    out->features.motion.altitude_m = value->motion()->altitude_m();
    out->features.motion.acceleration_m_per_s2 = value->motion()->acceleration_m_per_s2();
    out->features.motion.turn_radius_m = value->motion()->turn_radius_m();
    out->features.motion.is_straight = value->motion()->is_straight();
    out->features.motion.quality = value->motion()->quality();
  }
  if (value->polarization() != nullptr) {
    out->features.polarization.valid = value->polarization()->valid();
    out->features.polarization.energy_difference_db =
        value->polarization()->energy_difference_db();
    out->features.polarization.relative_difference_db =
        value->polarization()->relative_difference_db();
    out->features.polarization.energy_sum_db = value->polarization()->energy_sum_db();
    out->features.polarization.quality = value->polarization()->quality();
  }
  if (value->range_profile() != nullptr) {
    out->features.range_profile.valid = value->range_profile()->valid();
    out->features.range_profile.length_m = value->range_profile()->length_m();
    out->features.range_profile.peak_count = value->range_profile()->peak_count();
    out->features.range_profile.peak_energy_concentration =
        value->range_profile()->peak_energy_concentration();
    out->features.range_profile.resolution_m = value->range_profile()->resolution_m();
    out->features.range_profile.quality = value->range_profile()->quality();
  }
  out->valid_feature_mask = value->valid_feature_mask();
  out->look_az_deg = value->look_az_deg();
  out->look_el_deg = value->look_el_deg();
  out->range_m = value->range_m();
  out->snr_db = value->snr_db();
  out->dwell_sec = value->dwell_sec();
  out->bandwidth_hz = value->bandwidth_hz();
  out->has_platform_position = value->has_platform_position();
  out->platform_position.x_m = value->platform_position_x_m();
  out->platform_position.y_m = value->platform_position_y_m();
  out->platform_position.z_m = value->platform_position_z_m();
  out->cycle_index = value->cycle_index();
  out->batch_id = value->batch_id();
}

flatbuffers::Offset<fb::RirTrackAttributionRecordV2> EncodeTrackAttribution(
    flatbuffers::FlatBufferBuilder* builder, const RirTrackAttributionRecord& value) {
  return fb::CreateRirTrackAttributionRecordV2(
      *builder, value.association_key, value.external_target_id,
      builder->CreateString(value.target_name), value.hit_count, value.position_enu_x_m,
      value.position_enu_y_m, value.position_enu_z_m, value.speed_m_per_s);
}

void DecodeTrackAttribution(const fb::RirTrackAttributionRecordV2* value,
                            RirTrackAttributionRecord* out) {
  if (value == nullptr) {
    return;
  }
  out->association_key = value->association_key();
  out->external_target_id = value->external_target_id();
  if (value->target_name() != nullptr) {
    out->target_name = value->target_name()->str();
  }
  out->hit_count = value->hit_count();
  out->position_enu_x_m = value->position_enu_x_m();
  out->position_enu_y_m = value->position_enu_y_m();
  out->position_enu_z_m = value->position_enu_z_m();
  out->speed_m_per_s = value->speed_m_per_s();
}

flatbuffers::Offset<fb::RirRecognitionCycleSummaryV2> EncodeSummary(
    flatbuffers::FlatBufferBuilder* builder, const RirRecognitionCycleSummary& value) {
  const flatbuffers::Offset<fb::RirDwellBudgetSummaryV2> dwell_budget =
      fb::CreateRirDwellBudgetSummaryV2(*builder, value.dwell_budget.scheduled_dwell_count,
                                        value.dwell_budget.executed_dwell_count,
                                        value.dwell_budget.dwell_budget_sec,
                                        value.dwell_budget.dwell_consumed_sec);
  return fb::CreateRirRecognitionCycleSummaryV2(
      *builder, value.participating_track_count, value.category_confirmed_count,
      value.model_confirmed_count, value.unknown_count, value.disabled_count,
      value.rcs_availability_rate, value.motion_availability_rate,
      value.polarization_availability_rate, value.range_profile_availability_rate,
      value.mean_confidence, value.mean_first_confirmation_sec, value.has_ground_truth,
      value.category_accuracy, value.model_accuracy, dwell_budget);
}

void DecodeSummary(const fb::RirRecognitionCycleSummaryV2* value, RirRecognitionCycleSummary* out) {
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
  if (value->dwell_budget() != nullptr) {
    out->dwell_budget.scheduled_dwell_count = value->dwell_budget()->scheduled_dwell_count();
    out->dwell_budget.executed_dwell_count = value->dwell_budget()->executed_dwell_count();
    out->dwell_budget.dwell_budget_sec = value->dwell_budget()->dwell_budget_sec();
    out->dwell_budget.dwell_consumed_sec = value->dwell_budget()->dwell_consumed_sec();
  }
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
  // 出口①特征量测（加性向量，总是写入——空列表编码为存在但为空）。
  std::vector<flatbuffers::Offset<fb::RirFeatureMeasurementRecordV1>> feature_measurements;
  feature_measurements.reserve(record.result.output_frame.feature_measurements.size());
  for (std::size_t i = 0U; i < record.result.output_frame.feature_measurements.size(); ++i) {
    feature_measurements.push_back(
        EncodeFeatureMeasurement(&builder, record.result.output_frame.feature_measurements[i]));
  }
  const flatbuffers::Offset<fb::RirOutputFrameV1> output_frame = fb::CreateRirOutputFrameV1(
      builder, record.result.output_frame.input_cycle_index, record.result.output_frame.batch_id,
      builder.CreateVector(outputs), builder.CreateVector(feature_measurements));

  flatbuffers::Offset<fb::RirRecognitionCycleSummaryV2> summary_offset;
  if (record.result.has_recognition_summary) {
    summary_offset = EncodeSummary(&builder, record.result.recognition_summary);
  }

  // 航迹归属视图（结果层加性向量，总是写入）。
  std::vector<flatbuffers::Offset<fb::RirTrackAttributionRecordV2>> attributions;
  attributions.reserve(record.result.track_attributions.size());
  for (std::size_t i = 0U; i < record.result.track_attributions.size(); ++i) {
    attributions.push_back(
        EncodeTrackAttribution(&builder, record.result.track_attributions[i]));
  }

  const flatbuffers::Offset<fb::RirCycleResultV2> result = fb::CreateRirCycleResultV2(
      builder, record.result.input_cycle_index, static_cast<int>(record.result.status),
      static_cast<int>(record.result.abort_reason), output_frame,
      record.result.has_recognition_summary, summary_offset, record.result.designated_target_id,
      record.result.designation_active, record.result.designation_reverted_to_scan,
      static_cast<int>(record.result.designation_revert_reason),
      record.result.dwell_center_deg.az_deg, record.result.dwell_center_deg.el_deg,
      builder.CreateVector(attributions), EncodeRfV2Scene(&builder, record.result.emission_frame));

  const flatbuffers::Offset<fb::RirSessionReplayStateV2> session_state =
      fb::CreateRirSessionReplayStateV2(
          builder, builder.CreateString(record.session_state.active_database_version),
          record.session_state.detection_random_seed);

  const flatbuffers::Offset<fb::RirCycleReplayRecordV2> root =
      fb::CreateRirCycleReplayRecordV2(builder, result, session_state);
  builder.Finish(root, fb::RirCycleReplayRecordV2Identifier());
  return std::string(reinterpret_cast<const char*>(builder.GetBufferPointer()), builder.GetSize());
}

bool DecodeCycleReplayRecordFlatbuffer(const std::string& payload_bytes,
                                       RirCycleReplayRecord* record, std::string* error) {
  if (record == nullptr) {
    if (error != nullptr) {
      *error = "record output must be non-null";
    }
    return false;
  }
  const std::uint8_t* payload = reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  if (!flatbuffers::BufferHasIdentifier(payload, fb::RirCycleReplayRecordV2Identifier())) {
    if (error != nullptr) {
      *error = "rir_replay v1 record is not supported: phase 2-S uses the destructive v2 schema";
    }
    return false;
  }
  flatbuffers::Verifier verifier(payload, payload_bytes.size());
  if (!fb::VerifyRirCycleReplayRecordV2Buffer(verifier)) {
    if (error != nullptr) {
      *error = "flatbuffer verification failed";
    }
    return false;
  }
  const fb::RirCycleReplayRecordV2* root = fb::GetRirCycleReplayRecordV2(payload_bytes.data());
  if (root == nullptr) {
    if (error != nullptr) {
      *error = "root table missing";
    }
    return false;
  }
  RirCycleReplayRecord candidate;
  const fb::RirCycleResultV2* result = root->result();
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
  candidate.result.designated_target_id = result->designated_target_id();
  candidate.result.designation_active = result->designation_active();
  candidate.result.designation_reverted_to_scan = result->designation_reverted_to_scan();
  const int revert_reason_raw = result->designation_revert_reason();
  if (revert_reason_raw <
          static_cast<int>(RirDesignationRevertReason::kNone) ||
      revert_reason_raw > static_cast<int>(RirDesignationRevertReason::kAcquisitionTimeout)) {
    if (error != nullptr) {
      *error = "invalid designation revert reason";
    }
    return false;
  }
  candidate.result.designation_revert_reason =
      static_cast<RirDesignationRevertReason>(revert_reason_raw);
  candidate.result.dwell_center_deg.az_deg = result->dwell_center_az_deg();
  candidate.result.dwell_center_deg.el_deg = result->dwell_center_el_deg();
  candidate.result.emission_frame = DecodeRfV2Scene(result->emission_frame());

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
    // 出口①特征量测：旧记录字段缺席（null）→ 保持空（加性兼容）。
    const auto* feature_measurements = output_frame->feature_measurements();
    if (feature_measurements != nullptr) {
      candidate.result.output_frame.feature_measurements.reserve(feature_measurements->size());
      for (const fb::RirFeatureMeasurementRecordV1* measurement : *feature_measurements) {
        if (measurement == nullptr) {
          continue;
        }
        RirFeatureMeasurementRecord decoded;
        DecodeFeatureMeasurement(measurement, &decoded);
        candidate.result.output_frame.feature_measurements.push_back(decoded);
      }
    }
  }

  // 航迹归属视图：旧记录字段缺席（null）→ 保持空。
  const auto* attributions = result->track_attributions();
  if (attributions != nullptr) {
    candidate.result.track_attributions.reserve(attributions->size());
    for (const fb::RirTrackAttributionRecordV2* attribution : *attributions) {
      if (attribution == nullptr) {
        continue;
      }
      RirTrackAttributionRecord decoded;
      DecodeTrackAttribution(attribution, &decoded);
      candidate.result.track_attributions.push_back(decoded);
    }
  }

  const fb::RirSessionReplayStateV2* session_state = root->session_state();
  if (session_state != nullptr) {
    if (session_state->active_database_version() != nullptr) {
      candidate.session_state.active_database_version =
          session_state->active_database_version()->str();
    }
    candidate.session_state.detection_random_seed = session_state->detection_random_seed();
  }

  *record = std::move(candidate);
  return true;
}

}  // namespace session
}  // namespace remote_identification_radar
