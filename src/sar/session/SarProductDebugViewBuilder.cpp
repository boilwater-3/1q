#include "1q/sar/session/SarProductDebugView.h"

#include "1q/sar/session/SarCycleInput.h"
#include "sar/session/SarDiagnosticUtils.h"

namespace sar {
namespace session {

SarProductDebugView SarProductDebugViewBuilder::Build(const SarCycleInput& input,
                                                      const SarCycleResult& result) {
  SarProductDebugView view;
  view.input_cycle_index = result.input_cycle_index;
  view.output_cycle_index = result.output_frame.cycle_index;
  view.executed_this_cycle = (result.status == SarCycleStatus::kCompleted);
  view.abort_reason = AbortReasonToDiagnosticCode(result.abort_reason);
  view.completed_stage = result.output_frame.completed_stage;
  view.has_raw_echo = result.output_frame.has_raw_echo;
  view.has_range_compressed_echo = result.output_frame.has_range_compressed_echo;
  view.has_l1_image = result.output_frame.has_l1_image;
  view.has_l3_bp_image = result.output_frame.has_l3_bp_image;
  view.has_focused_pixels =
      !result.focused_image.real_values.empty() || !result.focused_image.imaginary_values.empty();
  view.estimated_snr_db = result.output_frame.estimated_snr_db;
  view.range_sample_count = result.output_frame.range_sample_count;
  view.azimuth_pulse_count = result.output_frame.azimuth_pulse_count;
  view.issues = result.issues;
  view.point_targets.reserve(input.point_targets.size());
  for (const SarPointTarget& target : input.point_targets) {
    SarDebugPointTarget debug_target;
    debug_target.target_id = target.target_id;
    debug_target.target_name = target.target_name;
    debug_target.radar_cross_section_dbsm = target.radar_cross_section_dbsm;
    view.point_targets.push_back(debug_target);
  }
  return view;
}

}  // namespace session
}  // namespace sar
