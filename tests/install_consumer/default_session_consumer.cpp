/**
 * @file default_session_consumer.cpp
 * @brief 验证安装后的默认 RadarSession 路径可被外部工程编译链接。
 */

#include <cstddef>

#include "1q/airborne_radar/common/config/ConfigPresets.h"
#include "1q/airborne_radar/common/output/TrackOutputQueries.h"
#include "1q/airborne_radar/core/context/RadarCycleInput.h"
#include "1q/airborne_radar/core/context/RadarInputValidation.h"
#include "1q/airborne_radar/core/session/RadarSession.h"
#include "1q/api.hpp"

int main() {
  airborne_radar::core::session::RadarSessionConfig config =
      airborne_radar::common::config::MakeDefaultRadarSessionConfig();
  airborne_radar::core::session::RadarSession session(config);

  airborne_radar::core::context::RadarCycleInput input;
  input.dt_sec = 1.0f;

  const std::vector<airborne_radar::core::context::ValidationIssue> issues =
      airborne_radar::core::context::ValidateRadarCycleInput(input);
  if (airborne_radar::core::context::HasValidationError(issues)) {
    return 1;
  }

  const airborne_radar::core::session::RadarCycleResult result = session.StepWithResult(input);
  const std::size_t confirmed_tracks = airborne_radar::common::output::CountTracksByStatus(
      result.track_output_frame, airborne_radar::common::model::DecisionTrackStatus::kConfirmed);

  return confirmed_tracks > result.track_output_frame.tracks.size() ? 1 : 0;
}
