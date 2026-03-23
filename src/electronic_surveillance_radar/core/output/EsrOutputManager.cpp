#include "electronic_surveillance_radar/core/output/EsrOutputManager.h"

#include <cstddef>

namespace electronic_surveillance_radar {
namespace core {
namespace output {

namespace {

/**
 * @brief 统计真值关联记录中的匹配命中数量。
 * @param[in] associations 真值关联记录列表。
 * @return 匹配成功的记录数量。
 */
std::size_t CountMatched(
    const common::TruthAssociationRecordList& associations) {
  std::size_t matched_count = 0U;
  for (std::size_t i = 0; i < associations.size(); ++i) {
    if (associations[i].matched) {
      ++matched_count;
    }
  }
  return matched_count;
}

}  // namespace

common::EsrOutputFrame EsrOutputManager::BuildOutputFrame(
    std::uint32_t cycle_index, std::uint64_t batch_id,
    const pipeline::InterceptCycleResult& cycle_result) const {
  common::EsrOutputFrame frame = BuildEmptyFrame(cycle_index, batch_id);
  frame.observation_output.observations = cycle_result.observations;
  frame.emitter_output.hypotheses = cycle_result.emitter_hypotheses;
  frame.truth_evaluation_output.associations = cycle_result.truth_associations;
  frame.truth_evaluation_output.total_observation_count =
      cycle_result.observations.size();
  frame.truth_evaluation_output.matched_count =
      CountMatched(cycle_result.truth_associations);
  return frame;
}

common::EsrOutputFrame EsrOutputManager::BuildEmptyFrame(
    std::uint32_t cycle_index, std::uint64_t batch_id) const {
  common::EsrOutputFrame frame;
  frame.observation_output.cycle_index = cycle_index;
  frame.observation_output.batch_id = batch_id;
  frame.emitter_output.cycle_index = cycle_index;
  frame.emitter_output.batch_id = batch_id;
  frame.truth_evaluation_output.cycle_index = cycle_index;
  frame.truth_evaluation_output.batch_id = batch_id;
  frame.truth_evaluation_output.total_observation_count = 0U;
  frame.truth_evaluation_output.matched_count = 0U;
  return frame;
}

}  // namespace output
}  // namespace core
}  // namespace electronic_surveillance_radar
