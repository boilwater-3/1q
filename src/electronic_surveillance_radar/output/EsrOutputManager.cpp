#include "electronic_surveillance_radar/output/EsrOutputManager.h"

#include <cstddef>

#include "common/output/OutputFrameUtils.h"

namespace electronic_surveillance_radar {
namespace output {

output::EsrOutputFrame EsrOutputManager::BuildOutputFrame(
    std::uint32_t cycle_index, std::uint64_t batch_id,
    const extension::InterceptCycleResult& cycle_result) const {
  output::EsrOutputFrame frame = BuildEmptyFrame(cycle_index, batch_id);
  frame.observation_output.observations = cycle_result.observations;
  frame.emitter_output.hypotheses = cycle_result.emitter_hypotheses;
  frame.truth_evaluation_output.associations = cycle_result.truth_associations;
  frame.truth_evaluation_output.total_observation_count = cycle_result.observations.size();
  frame.truth_evaluation_output.matched_count = oneq::internal::output::CountMatching(
      cycle_result.truth_associations,
      [](const output::TruthAssociationRecord& association) { return association.matched; });
  return frame;
}

output::EsrOutputFrame EsrOutputManager::BuildEmptyFrame(std::uint32_t cycle_index,
                                                         std::uint64_t batch_id) const {
  output::EsrOutputFrame frame;
  oneq::internal::output::SetCycleAndBatch(frame.observation_output, cycle_index, batch_id);
  oneq::internal::output::SetCycleAndBatch(frame.emitter_output, cycle_index, batch_id);
  oneq::internal::output::SetCycleAndBatch(frame.truth_evaluation_output, cycle_index, batch_id);
  frame.truth_evaluation_output.total_observation_count = 0U;
  frame.truth_evaluation_output.matched_count = 0U;
  return frame;
}

}  // namespace output

}  // namespace electronic_surveillance_radar
