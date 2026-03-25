#include "electronic_surveillance_radar/core/output/EsrOutputManager.h"

#include <cstddef>

#include "common/output/OutputFrameUtils.h"

namespace electronic_surveillance_radar {
namespace core {
namespace output {

common::EsrOutputFrame EsrOutputManager::BuildOutputFrame(
    std::uint32_t cycle_index, std::uint64_t batch_id,
    const pipeline::InterceptCycleResult& cycle_result) const {
  common::EsrOutputFrame frame = BuildEmptyFrame(cycle_index, batch_id);
  frame.observation_output.observations = cycle_result.observations;
  frame.emitter_output.hypotheses = cycle_result.emitter_hypotheses;
  frame.truth_evaluation_output.associations = cycle_result.truth_associations;
  frame.truth_evaluation_output.total_observation_count = cycle_result.observations.size();
  frame.truth_evaluation_output.matched_count = oneq::internal::output::CountMatching(
      cycle_result.truth_associations,
      [](const common::TruthAssociationRecord& association) { return association.matched; });
  return frame;
}

common::EsrOutputFrame EsrOutputManager::BuildEmptyFrame(std::uint32_t cycle_index,
                                                         std::uint64_t batch_id) const {
  common::EsrOutputFrame frame;
  oneq::internal::output::SetCycleAndBatch(frame.observation_output, cycle_index, batch_id);
  oneq::internal::output::SetCycleAndBatch(frame.emitter_output, cycle_index, batch_id);
  oneq::internal::output::SetCycleAndBatch(frame.truth_evaluation_output, cycle_index, batch_id);
  frame.truth_evaluation_output.total_observation_count = 0U;
  frame.truth_evaluation_output.matched_count = 0U;
  return frame;
}

}  // namespace output
}  // namespace core
}  // namespace electronic_surveillance_radar
