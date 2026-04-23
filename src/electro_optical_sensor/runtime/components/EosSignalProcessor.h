#ifndef ELECTRO_OPTICAL_SENSOR_RUNTIME_COMPONENTS_EOS_SIGNAL_PROCESSOR_H_
#define ELECTRO_OPTICAL_SENSOR_RUNTIME_COMPONENTS_EOS_SIGNAL_PROCESSOR_H_

#include "1q/electro_optical_sensor/extension/IEosPipeline.h"

namespace electro_optical_sensor {
namespace runtime {
namespace components {

class EosSignalProcessor {
 public:
  explicit EosSignalProcessor(extension::IEosPipeline& pipeline);

  extension::EosPipelineRuntimeState CaptureRuntimeState() const;

  bool RestoreRuntimeState(const extension::EosPipelineRuntimeState& state) const;

  extension::EosPipelineExecuteResult Execute(
      const ::electro_optical_sensor::session::EosCycleInput& input) const;

 private:
  extension::IEosPipeline& pipeline_;
};

}  // namespace components
}  // namespace runtime
}  // namespace electro_optical_sensor

#endif  // ELECTRO_OPTICAL_SENSOR_RUNTIME_COMPONENTS_EOS_SIGNAL_PROCESSOR_H_
