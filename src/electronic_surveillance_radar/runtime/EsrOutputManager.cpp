#include "electronic_surveillance_radar/runtime/EsrOutputManager.h"

#include "common/output/OutputFrameUtils.h"

namespace electronic_surveillance_radar {
namespace output {

void EsrOutputManager::StampOutputFrame(std::uint32_t cycle_index, std::uint64_t batch_id,
                                         session::EsrOutputFrame& frame) const {
  oneq::internal::output::SetCycleAndBatch(frame, cycle_index, batch_id);
}

session::EsrOutputFrame EsrOutputManager::BuildEmptyFrame(std::uint32_t cycle_index,
                                                         std::uint64_t batch_id) const {
  session::EsrOutputFrame frame;
  oneq::internal::output::SetCycleAndBatch(frame, cycle_index, batch_id);
  return frame;
}

}  // namespace output

}  // namespace electronic_surveillance_radar
