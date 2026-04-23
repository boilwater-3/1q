#include "electronic_surveillance_radar/runtime/components/EsrOutputFormatter.h"

#include <cstddef>

#include "electronic_surveillance_radar/runtime/EsrCycleTelemetryLogger.h"

namespace electronic_surveillance_radar {
namespace runtime {
namespace components {

EsrOutputFormatter::EsrOutputFormatter(output::EsrOutputManager& output_manager)
    : output_manager_(output_manager) {}

output::EsrOutputFrame EsrOutputFormatter::BuildOutputFrame(
    const oneq::internal::runtime::RuntimeCycleStamp& stamp,
    const extension::InterceptCycleResult& intercept_result) const {
  return output_manager_.BuildOutputFrame(stamp.cycle_index, stamp.batch_id, intercept_result);
}

output::EsrOutputFrame EsrOutputFormatter::BuildEmptyFrame(
    const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
  return output_manager_.BuildEmptyFrame(stamp.cycle_index, stamp.batch_id);
}

void EsrOutputFormatter::LogCycleSummary(
    const session::EsrCycleInput& cycle_input,
    const oneq::internal::runtime::RuntimeCycleStamp& stamp,
    const extension::InterceptCycleResult& intercept_result,
    const output::EsrOutputFrame& output_frame) const {
  std::size_t matched_truth_count = 0U;
  for (std::size_t i = 0; i < output_frame.truth_evaluation_output.associations.size(); ++i) {
    if (output_frame.truth_evaluation_output.associations[i].matched) {
      ++matched_truth_count;
    }
  }

  const runtime::EsrCycleTelemetryPayload payload(
      stamp, cycle_input.scene.size(), intercept_result.raw_observation_count,
      output_frame.observation_output.observations.size(), intercept_result.cluster_count,
      output_frame.emitter_output.hypotheses.size(),
      output_frame.truth_evaluation_output.associations.size(), matched_truth_count, true);
  runtime::EsrCycleTelemetryLogger::LogCycleSummary(payload);
}

}  // namespace components
}  // namespace runtime
}  // namespace electronic_surveillance_radar
