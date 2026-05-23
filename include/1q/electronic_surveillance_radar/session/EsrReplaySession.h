/**
 * @file EsrReplaySession.h
 * @brief Provides an ESR replay entry point backed by replay trace events.
 */

#ifndef ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_REPLAY_SESSION_H_
#define ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_REPLAY_SESSION_H_

#include <string>

#include "1q/api.hpp"
#include "1q/replay/ReplayTrace.h"

namespace electronic_surveillance_radar {
namespace session {

struct ONEQ_API EsrReplaySessionResult {
  oneq::replay::ReplayTraceReplayReport report{};
  oneq::replay::ReplayTracePlaybackResult playback{};
  bool ok{false};
  bool reached_failure_marker{false};
  std::string failure_marker_payload{};
  oneq::replay::ReplayTraceFailure failure_marker_data{};
  std::string first_error{};
};

ONEQ_API EsrReplaySessionResult ReplayEsrTrace(const std::string& trace_dir);

}  // namespace session
}  // namespace electronic_surveillance_radar

#endif  // ONEQ_ELECTRONIC_SURVEILLANCE_RADAR_SESSION_ESR_REPLAY_SESSION_H_
