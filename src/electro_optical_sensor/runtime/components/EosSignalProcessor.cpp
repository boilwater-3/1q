#include "electro_optical_sensor/runtime/components/EosSignalProcessor.h"

namespace electro_optical_sensor {
namespace runtime {
namespace components {

EosSignalProcessor::EosSignalProcessor(extension::IEosPipeline& pipeline) : pipeline_(pipeline) {}

extension::EosPipelineRuntimeState EosSignalProcessor::CaptureRuntimeState() const {
  return pipeline_.CaptureRuntimeState();
}

bool EosSignalProcessor::RestoreRuntimeState(
    const extension::EosPipelineRuntimeState& state) const {
  return pipeline_.RestoreRuntimeState(state);
}

extension::EosPipelineExecuteResult EosSignalProcessor::Execute(
    const ::electro_optical_sensor::session::EosCycleInput& input) const {
  return pipeline_.Execute(input);
}

}  // namespace components
}  // namespace runtime
}  // namespace electro_optical_sensor
