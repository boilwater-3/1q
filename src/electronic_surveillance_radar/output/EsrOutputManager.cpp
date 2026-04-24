#include "electronic_surveillance_radar/output/EsrOutputManager.h"

#include "common/output/OutputFrameUtils.h"

namespace electronic_surveillance_radar {
namespace output {

void EsrOutputManager::StampOutputFrame(std::uint32_t cycle_index, std::uint64_t batch_id,
                                         output::EsrOutputFrame& frame) const {
  oneq::internal::output::SetCycleAndBatch(frame, cycle_index, batch_id);
}

output::EsrOutputFrame EsrOutputManager::BuildEmptyFrame(std::uint32_t cycle_index,
                                                         std::uint64_t batch_id) const {
  output::EsrOutputFrame frame;
  oneq::internal::output::SetCycleAndBatch(frame, cycle_index, batch_id);
  return frame;
}

}  // namespace output

}  // namespace electronic_surveillance_radar
