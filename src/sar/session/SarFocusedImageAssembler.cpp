#include "sar/session/SarFocusedImageAssembler.h"

namespace sar {
namespace session {

void InitializeOutputFrameMetadata(const config::SarSessionConfig& config, SarOutputFrame* frame) {
  frame->range_sample_count = config.mission.range_sample_count;
  frame->azimuth_pulse_count = config.mission.azimuth_pulse_count;
  frame->center_slant_range_m = config.mission.nominal_slant_range_m;
  frame->estimated_snr_db = 0.0;
}

void MarkRawEchoStage(SarOutputFrame* frame, double estimated_snr_db) {
  frame->completed_stage = SarProcessingStage::kRawEcho;
  frame->has_raw_echo = true;
  frame->estimated_snr_db = estimated_snr_db;
}

}  // namespace session
}  // namespace sar
