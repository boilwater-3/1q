#include "electronic_surveillance_radar/runtime/components/EsrOutputFormatter.h"

namespace electronic_surveillance_radar {
namespace runtime {
namespace components {

EsrOutputFormatter::EsrOutputFormatter(output::EsrOutputManager& output_manager)
    : output_manager_(output_manager) {}

session::EsrOutputFrame EsrOutputFormatter::BuildEmptyFrame(
    const oneq::internal::runtime::RuntimeCycleStamp& stamp) const {
  return output_manager_.BuildEmptyFrame(stamp.cycle_index, stamp.batch_id);
}

}  // namespace components
}  // namespace runtime
}  // namespace electronic_surveillance_radar
