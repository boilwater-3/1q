#include "electronic_surveillance_radar/pipeline/MutableEsrContext.h"

namespace electronic_surveillance_radar {
namespace pipeline {

void MutableEsrContext::BeginCycle(const session::EsrCycleInput& input,
                                   const session::EsrEnvironmentSnapshot& environment_snapshot,
                                   const extension::InterceptPipelineConfig& pipeline_config,
                                   const extension::InterceptRuntimeConfig& runtime_config) {
  cycle_input_ = &input;
  cycle_index_ = input.cycle_index;
  cycle_start_time_s_ = input.cycle_start_time_s;
  dt_sec_ = input.dt_sec;
  platform_entity_id_ = input.platform_entity_id;
  has_platform_ecef_kinematics_ = input.has_platform_ecef_kinematics;
  platform_position_ecef_m_ = input.platform_position_ecef_m;
  platform_velocity_ecef_mps_ = input.platform_velocity_ecef_mps;
  platform_attitude_deg_ = input.platform_attitude_deg;
  interference_ = input.interference;
  environment_snapshot_ = environment_snapshot;
  pipeline_config_ = pipeline_config;
  runtime_config_ = runtime_config;
}

std::uint32_t MutableEsrContext::GetCycleIndex() const { return cycle_index_; }

const session::EsrCycleInput& MutableEsrContext::GetCycleInput() const { return *cycle_input_; }

double MutableEsrContext::GetCycleStartTimeSec() const { return cycle_start_time_s_; }

float MutableEsrContext::GetCycleDeltaTimeSec() const { return dt_sec_; }

const oneq::coordinate::EulerAnglesDeg& MutableEsrContext::GetPlatformAttitude() const {
  return platform_attitude_deg_;
}

std::uint64_t MutableEsrContext::GetPlatformEntityId() const { return platform_entity_id_; }

bool MutableEsrContext::HasPlatformEcefKinematics() const {
  return has_platform_ecef_kinematics_;
}

const oneq::coordinate::EcefPositionM& MutableEsrContext::GetPlatformPositionEcefM() const {
  return platform_position_ecef_m_;
}

const oneq::coordinate::EcefVelocityMps& MutableEsrContext::GetPlatformVelocityEcefMps() const {
  return platform_velocity_ecef_mps_;
}

const oneq::electromagnetics::RfEmissionFrame& MutableEsrContext::GetInterference() const {
  return interference_;
}

const session::EsrEnvironmentSnapshot& MutableEsrContext::GetEnvironmentSnapshot() const {
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
