#include "electronic_surveillance_radar/pipeline/MutableEsrContext.h"

#include <cstddef>

namespace electronic_surveillance_radar {
namespace pipeline {

namespace {

model::EmitterTruthState ToModelEmitter(const session::EsrSceneEmitter& input) {
  model::EmitterTruthState out{};
  out.emitter_id = input.emitter_id;
  out.pose = input.pose;
  out.carrier_hz = input.carrier_hz;
  out.bandwidth_hz = input.bandwidth_hz;
  out.tx_power_w = input.tx_power_w;
  out.pulse_width_s = input.pulse_width_s;
  out.pri_s = input.pri_s;
  out.beam_state.center_az_deg = input.beam_state.center_az_deg;
  out.beam_state.center_el_deg = input.beam_state.center_el_deg;
  out.beam_state.az_beamwidth_deg = input.beam_state.az_beamwidth_deg;
  out.beam_state.el_beamwidth_deg = input.beam_state.el_beamwidth_deg;
  out.beam_state.beam_state_valid = input.beam_state.beam_state_valid;
  out.is_emitting = input.is_emitting;
  return out;
}

model::EmitterTruthStateList ToModelEmitters(const session::EsrSceneEmitterList& inputs) {
  model::EmitterTruthStateList outputs;
  outputs.reserve(inputs.size());
  for (std::size_t i = 0; i < inputs.size(); ++i) {
    outputs.push_back(ToModelEmitter(inputs[i]));
  }
  return outputs;
}

}  // namespace

void MutableEsrContext::BeginCycle(const session::EsrCycleInput& input,
                                   const environment::EsrEnvironmentSnapshot& environment_snapshot,
                                   const extension::InterceptPipelineConfig& pipeline_config,
                                   const extension::InterceptRuntimeConfig& runtime_config) {
  cycle_index_ = input.cycle_index;
  dt_sec_ = input.dt_sec;
  platform_pose_ = input.platform_pose;
  scene_emitters_ = ToModelEmitters(input.scene);
  environment_snapshot_ = environment_snapshot;
  pipeline_config_ = pipeline_config;
  runtime_config_ = runtime_config;
}

std::uint32_t MutableEsrContext::GetCycleIndex() const { return cycle_index_; }

float MutableEsrContext::GetCycleDeltaTimeSec() const { return dt_sec_; }

const model::EsrPoseState& MutableEsrContext::GetPlatformPose() const { return platform_pose_; }

const model::EmitterTruthStateList& MutableEsrContext::GetSceneEmitters() const {
  return scene_emitters_;
}

const environment::EsrEnvironmentSnapshot& MutableEsrContext::GetEnvironmentSnapshot() const {
  return environment_snapshot_;
}

const extension::InterceptPipelineConfig& MutableEsrContext::GetPipelineConfig() const {
  return pipeline_config_;
}

const extension::InterceptRuntimeConfig& MutableEsrContext::GetRuntimeConfig() const {
  return runtime_config_;
}

}  // namespace pipeline

}  // namespace electronic_surveillance_radar
