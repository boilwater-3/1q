#include "electronic_surveillance_radar/pipeline/MutableEsrContext.h"

namespace electronic_surveillance_radar {
namespace pipeline {

void MutableEsrContext::BeginCycle(const session::EsrCycleInput& input,
                                   const environment::EsrEnvironmentSnapshot& environment_snapshot,
                                   const extension::InterceptPipelineConfig& pipeline_config,
                                   const extension::InterceptRuntimeConfig& runtime_config) {
  cycle_index_ = input.cycle_index;
  dt_sec_ = input.dt_sec;
  platform_pose_ = input.platform_pose;
  scene_emitters_ = input.scene.emitters;
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
