#include "airborne_radar/session/RadarReplayFlatbufferCodec.h"

#include <cstdint>
#include <string>
#include <vector>

#include "airborne_radar/session/generated/airborne_radar_replay_generated.h"
#include "airborne_radar/session/generated/airborne_radar_scene_replay_generated.h"

namespace airborne_radar {
namespace session {
namespace {

namespace fb = oneq::replay::airborne_radar::fb;
namespace scene_fb = oneq::replay::airborne_radar::scene::fb;

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

flatbuffers::Offset<scene_fb::AtmosphericPhysicsConfig> EncodeAtmosphericPhysicsConfig(
    flatbuffers::FlatBufferBuilder* builder,
    const environment::AtmosphericPhysicsConfig& value) {
  return scene_fb::CreateAtmosphericPhysicsConfig(
      *builder, value.enable_physical_model, value.pressure_hpa,
      value.temperature_k, value.relative_humidity);
}

flatbuffers::Offset<scene_fb::AtmosphericDerivedContext> EncodeAtmosphericDerivedContext(
    flatbuffers::FlatBufferBuilder* builder,
    const environment::AtmosphericDerivedContext& value) {
  return scene_fb::CreateAtmosphericDerivedContext(
      *builder, value.has_simulation_unix_seconds, value.simulation_unix_seconds,
      value.solar_flux_f107a, value.solar_flux_f107, value.geomagnetic_ap);
}

flatbuffers::Offset<scene_fb::VegetationScatterPhysicsConfig>
EncodeVegetationScatterPhysicsConfig(
    flatbuffers::FlatBufferBuilder* builder,
    const environment::VegetationScatterPhysicsConfig& value) {
  return scene_fb::CreateVegetationScatterPhysicsConfig(
      *builder, static_cast<int>(value.cover_profile), value.enable_physical_model);
}

flatbuffers::Offset<scene_fb::JammerEmitterState> EncodeJammerEmitterState(
    flatbuffers::FlatBufferBuilder* builder,
    const environment::JammerEmitterState& value) {
  return scene_fb::CreateJammerEmitterState(
      *builder, static_cast<int>(value.technique), value.power_db, value.js_db,
      value.has_direction_deg, value.azimuth_deg, value.elevation_deg,
      value.angular_span_deg, value.confidence);
}

environment::AtmosphericPhysicsConfig DecodeAtmosphericPhysicsConfig(
    const scene_fb::AtmosphericPhysicsConfig* value) {
  environment::AtmosphericPhysicsConfig result;
  if (value != nullptr) {
    result.enable_physical_model = value->enable_physical_model();
    result.pressure_hpa = value->pressure_hpa();
    result.temperature_k = value->temperature_k();
    result.relative_humidity = value->relative_humidity();
  }
  return result;
}

environment::AtmosphericDerivedContext DecodeAtmosphericDerivedContext(
    const scene_fb::AtmosphericDerivedContext* value) {
  environment::AtmosphericDerivedContext result;
  if (value != nullptr) {
    result.has_simulation_unix_seconds = value->has_simulation_unix_seconds();
    result.simulation_unix_seconds = value->simulation_unix_seconds();
    result.solar_flux_f107a = value->solar_flux_f107a();
    result.solar_flux_f107 = value->solar_flux_f107();
    result.geomagnetic_ap = value->geomagnetic_ap();
  }
  return result;
}

environment::VegetationScatterPhysicsConfig DecodeVegetationScatterPhysicsConfig(
    const scene_fb::VegetationScatterPhysicsConfig* value) {
  environment::VegetationScatterPhysicsConfig result;
  if (value != nullptr) {
    result.cover_profile = static_cast<environment::VegetationCoverProfile>(
        value->cover_profile());
    result.enable_physical_model = value->enable_physical_model();
  }
  return result;
}

environment::JammerEmitterState DecodeJammerEmitterState(
    const scene_fb::JammerEmitterState* value) {
  environment::JammerEmitterState result;
  if (value != nullptr) {
    result.technique =
        static_cast<environment::JammingTechnique>(value->technique());
    result.power_db = value->power_db();
    result.js_db = value->js_db();
    result.has_direction_deg = value->has_direction_deg();
    result.azimuth_deg = value->azimuth_deg();
    result.elevation_deg = value->elevation_deg();
    result.angular_span_deg = value->angular_span_deg();
    result.confidence = value->confidence();
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

std::string EncodeSceneStateFlatbuffer(
    const environment::EnvironmentSceneState& scene_state) {
  flatbuffers::FlatBufferBuilder builder;
  std::vector<flatbuffers::Offset<scene_fb::JammerEmitterState> > emitters;
  emitters.reserve(scene_state.jammer_emitters.size());
  for (std::size_t i = 0; i < scene_state.jammer_emitters.size(); ++i) {
    emitters.push_back(
        EncodeJammerEmitterState(&builder, scene_state.jammer_emitters[i]));
  }

  const flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<scene_fb::JammerEmitterState> > >
      emitter_vector = builder.CreateVector(emitters);
  const flatbuffers::Offset<scene_fb::EnvironmentSceneState> root =
      scene_fb::CreateEnvironmentSceneState(
          builder,
          EncodeAtmosphericPhysicsConfig(&builder, scene_state.atmospheric_physics),
          EncodeAtmosphericDerivedContext(&builder, scene_state.atmospheric_context),
          EncodeVegetationScatterPhysicsConfig(
              &builder, scene_state.vegetation_scatter_physics),
          emitter_vector);
  builder.Finish(root, scene_fb::EnvironmentSceneStateIdentifier());

  const std::uint8_t* buffer = builder.GetBufferPointer();
  return std::string(reinterpret_cast<const char*>(buffer),
                     reinterpret_cast<const char*>(buffer) + builder.GetSize());
}

bool DecodeSceneStateFlatbuffer(const std::string& payload_bytes,
                                environment::EnvironmentSceneState* scene_state,
                                std::string* error) {
  if (scene_state == nullptr) {
    if (error != nullptr) {
      *error = "null EnvironmentSceneState output";
    }
    return false;
  }
  if (payload_bytes.empty()) {
    if (error != nullptr) {
      *error = "empty EnvironmentSceneState flatbuffers payload";
    }
    return false;
  }

  const std::uint8_t* data =
      reinterpret_cast<const std::uint8_t*>(payload_bytes.data());
  flatbuffers::Verifier verifier(data, payload_bytes.size());
  if (!scene_fb::VerifyEnvironmentSceneStateBuffer(verifier)) {
    if (error != nullptr) {
      *error = "invalid EnvironmentSceneState flatbuffers payload";
    }
    return false;
  }

  const scene_fb::EnvironmentSceneState* root =
      scene_fb::GetEnvironmentSceneState(data);
  scene_state->atmospheric_physics =
      DecodeAtmosphericPhysicsConfig(root->atmospheric_physics());
  scene_state->atmospheric_context =
      DecodeAtmosphericDerivedContext(root->atmospheric_context());
  scene_state->vegetation_scatter_physics = DecodeVegetationScatterPhysicsConfig(
      root->vegetation_scatter_physics());
  scene_state->jammer_emitters.clear();
  const flatbuffers::Vector<flatbuffers::Offset<scene_fb::JammerEmitterState> >*
      emitters = root->jammer_emitters();
  if (emitters != nullptr) {
    scene_state->jammer_emitters.reserve(emitters->size());
    for (flatbuffers::uoffset_t i = 0; i < emitters->size(); ++i) {
      scene_state->jammer_emitters.push_back(
          DecodeJammerEmitterState(emitters->Get(i)));
    }
  }
  return true;
}

}  // namespace session
}  // namespace airborne_radar
