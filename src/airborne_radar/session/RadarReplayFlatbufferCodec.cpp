#include "airborne_radar/session/RadarReplayFlatbufferCodec.h"

#include <cstdint>
#include <string>
#include <vector>

#include "airborne_radar/session/generated/airborne_radar_replay_generated.h"

namespace airborne_radar {
namespace session {
namespace {

namespace fb = oneq::replay::airborne_radar::fb;

flatbuffers::Offset<fb::Vector3f> EncodeVector3(
    flatbuffers::FlatBufferBuilder* builder,
    const oneq::foundation::Vector3f& value) {
  return fb::CreateVector3f(*builder, value.x, value.y, value.z);
}

flatbuffers::Offset<fb::Vector3f> EncodeVector3(
    flatbuffers::FlatBufferBuilder* builder,
    float x,
    float y,
    float z) {
  return fb::CreateVector3f(*builder, x, y, z);
}

flatbuffers::Offset<fb::EulerAnglesDeg> EncodeEulerAngles(
    flatbuffers::FlatBufferBuilder* builder,
    const oneq::foundation::EulerAnglesDeg& value) {
  return fb::CreateEulerAnglesDeg(*builder, value.yaw_deg, value.pitch_deg,
                                  value.roll_deg);
}

flatbuffers::Offset<fb::PoseState> EncodePoseState(
    flatbuffers::FlatBufferBuilder* builder,
    const oneq::foundation::PoseState& value) {
  return fb::CreatePoseState(*builder, EncodeVector3(builder, value.position_m),
                             EncodeVector3(builder, value.velocity_mps),
                             EncodeEulerAngles(builder, value.attitude_deg));
}

flatbuffers::Offset<fb::TargetFeature> EncodeTargetFeature(
    flatbuffers::FlatBufferBuilder* builder,
    const model::TargetFeature& value) {
  return fb::CreateTargetFeature(
      *builder, value.external_target_id,
      EncodeVector3(builder, value.current_track_velocity_x,
                    value.current_track_velocity_y,
                    value.current_track_velocity_z),
      value.current_track_speed, value.current_track_rcs, value.range_m,
      value.has_cartesian_position,
      EncodeVector3(builder, value.position_x, value.position_y, value.position_z),
      value.target_swerling_type);
}

oneq::foundation::Vector3f DecodeVector3(const fb::Vector3f* value) {
  oneq::foundation::Vector3f result;
  if (value != nullptr) {
    result.x = value->x();
    result.y = value->y();
    result.z = value->z();
  }
  return result;
}

oneq::foundation::EulerAnglesDeg DecodeEulerAngles(
    const fb::EulerAnglesDeg* value) {
  oneq::foundation::EulerAnglesDeg result;
  if (value != nullptr) {
    result.yaw_deg = value->yaw_deg();
    result.pitch_deg = value->pitch_deg();
    result.roll_deg = value->roll_deg();
  }
  return result;
}

oneq::foundation::PoseState DecodePoseState(const fb::PoseState* value) {
  oneq::foundation::PoseState result;
  if (value != nullptr) {
    result.position_m = DecodeVector3(value->position_m());
    result.velocity_mps = DecodeVector3(value->velocity_mps());
    result.attitude_deg = DecodeEulerAngles(value->attitude_deg());
  }
  return result;
}

model::TargetFeature DecodeTargetFeature(const fb::TargetFeature* value) {
  model::TargetFeature result;
  if (value != nullptr) {
    result.external_target_id = value->external_target_id();
    const oneq::foundation::Vector3f velocity = DecodeVector3(value->velocity_mps());
    result.current_track_velocity_x = velocity.x;
    result.current_track_velocity_y = velocity.y;
    result.current_track_velocity_z = velocity.z;
    result.current_track_speed = value->current_track_speed();
    result.current_track_rcs = value->current_track_rcs();
    result.range_m = value->range_m();
    result.has_cartesian_position = value->has_cartesian_position();
    const oneq::foundation::Vector3f position = DecodeVector3(value->position_m());
    result.position_x = position.x;
    result.position_y = position.y;
    result.position_z = position.z;
    result.target_swerling_type = value->target_swerling_type();
  }
  return result;
}

}  // namespace

std::string EncodeCycleInputFlatbuffer(const RadarCycleInput& input) {
  flatbuffers::FlatBufferBuilder builder;
  std::vector<flatbuffers::Offset<fb::TargetFeature> > targets;
  targets.reserve(input.target_features.size());
  for (std::size_t i = 0; i < input.target_features.size(); ++i) {
    targets.push_back(EncodeTargetFeature(&builder, input.target_features[i]));
  }

  const flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<fb::TargetFeature> > >
      target_vector = builder.CreateVector(targets);
  const flatbuffers::Offset<fb::RadarCycleInput> root =
      fb::CreateRadarCycleInput(builder, input.dt_sec,
                                EncodePoseState(&builder, input.platform_pose),
                                target_vector);
  builder.Finish(root, fb::RadarCycleInputIdentifier());

  const std::uint8_t* buffer = builder.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buffer),
                     reinterpret_cast<const char*>(buffer) + builder.GetSize());
}

bool DecodeCycleInputFlatbuffer(const std::string& payload_bytes,
                                RadarCycleInput* input,
                                std::string* error) {
  if (input == nullptr) {
    if (error != nullptr) {
      *error = "null RadarCycleInput output";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty RadarCycleInput flatbuffers payload";
    }
    return false;
  }

  const std::uint8_t* data =
      reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  if (!fb::VerifyRadarCycleInputBuffer(verifier)) {
    if (error != nullptr) {
      *error = "invalid RadarCycleInput flatbuffers payload";
    }
    return false;
  }

  const fb::RadarCycleInput* root = fb::GetRadarCycleInput(data);
  input->dt_sec = root->dt_sec();
  input->platform_pose = DecodePoseState(root->platform_pose());
  input->target_features.clear();
  const flatbuffers::Vector<flatbuffers::Offset<fb::TargetFeature> >* targets =
      root->target_features();
  if (targets != nullptr) {
    input->target_features.reserve(targets->size());
    for (flatbuffers::uoffset_t i = 0; i < targets->size(); ++i) {
      input->target_features.push_back(DecodeTargetFeature(targets->Get(i)));
    }
  }
  return true;
}

}  // namespace session
}  // namespace airborne_radar
