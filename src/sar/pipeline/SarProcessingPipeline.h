#ifndef ONEQ_SRC_SAR_PIPELINE_SAR_PROCESSING_PIPELINE_H_
#define ONEQ_SRC_SAR_PIPELINE_SAR_PROCESSING_PIPELINE_H_

#include <cstdint>
#include <deque>
#include <memory>

#include "1q/sar/config/SarSessionConfig.h"
#include "1q/sar/session/SarCycleInput.h"
#include "1q/sar/session/SarCycleResult.h"
#include "sar/geometry/SarGeometry.h"
#include "sar/runtime/PulseRingBuffer.h"

namespace sar {
namespace pipeline {

struct SarProcessingPipelineRuntimeState {
  const void* owner_identity{nullptr};
  std::uint32_t schema_version{0U};
  runtime::PulseRingBufferRuntimeState raw_pulse_buffer_state{};
  std::deque<geometry::PlatformPulseState> ideal_trajectory_buffer{};
  std::deque<geometry::PlatformPulseState> actual_trajectory_buffer{};
  std::uint64_t next_pulse_id{0U};
  double pulse_fraction_carry{0.0};
};

class SarProcessingPipeline {
 public:
  explicit SarProcessingPipeline(const config::SarSessionConfig& initial_config);
  ~SarProcessingPipeline();

  SarProcessingPipeline(const SarProcessingPipeline&) = delete;
  SarProcessingPipeline& operator=(const SarProcessingPipeline&) = delete;

  bool RunCycle(const config::SarSessionConfig& config, const session::SarCycleInput& input,
                session::SarCycleResult* result);

  SarProcessingPipelineRuntimeState CaptureRuntimeState() const;
  bool RestoreRuntimeState(const SarProcessingPipelineRuntimeState& state);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace pipeline
}  // namespace sar

#endif  // ONEQ_SRC_SAR_PIPELINE_SAR_PROCESSING_PIPELINE_H_
