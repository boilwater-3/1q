/**
 * @file RirAcceptanceRecords.h
 * @brief RIR 验收行拼装（检测链拆项、航迹/识别、调度、天线 CSV）。
 */

#ifndef ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_ACCEPTANCE_RECORDS_H_
#define ONEQ_SRC_REMOTE_IDENTIFICATION_RADAR_RUNTIME_RIR_ACCEPTANCE_RECORDS_H_

#include <cstdint>
#include <string>
#include <vector>

#include "1q/electromagnetics/RfScene.h"
#include "1q/remote_identification_radar/config/RirHardwareConfig.h"
#include "remote_identification_radar/dwell/RirDetectionCellResolver.h"

namespace remote_identification_radar {

namespace tracking {
struct RirTrackState;
struct RirAssociationResult;
}  // namespace tracking

namespace session {
struct RirRecognitionResult;
struct RirRecognitionCycleSummary;
struct RirFeatureMeasurementRecord;
}  // namespace session

namespace runtime {

struct RirDetectionAcceptInput {
  float sim_time_sec{0.0f};
  std::uint32_t cycle{0U};
  std::uint64_t target_id{0U};
  float range_m{0.0f};
  float look_az_deg{0.0f};
  float look_el_deg{0.0f};
  float rcs_m2{0.0f};
  float snr_db{0.0f};
  float pd{0.0f};
  bool detected{false};
  bool has_cell{false};
  float peak_gain_dbi{0.0f};
  float bw_az_deg{0.0f};
  float bw_el_deg{0.0f};
  double echo_power_dbw{0.0};
  dwell::RirDetectionCellResult cell{};
  config::hardware::RirSignalProcessingConfig gains{};
  double prf_hz{0.0};
  double center_frequency_hz{0.0};
  std::vector<oneq::electromagnetics::RfIncidentLinkResult> incident_links{};
};

void WriteRirAntennaPatternSummary(float sim_time_sec, std::uint32_t cycle, float peak_gain_dbi,
                                   float bw_az_deg, float bw_el_deg, const std::string& csv_path);

void WriteRirDetectionChain(const RirDetectionAcceptInput& input);

void WriteRirInterferenceLinks(
    float sim_time_sec, std::uint32_t cycle,
    const std::vector<oneq::electromagnetics::RfIncidentLinkResult>& links);

void WriteRirSearchDetections(float sim_time_sec, std::uint32_t cycle, float beam_az_deg,
                              float beam_el_deg, const std::string& found_targets);

void WriteRirAssociation(float sim_time_sec, std::uint32_t cycle,
                         const tracking::RirAssociationResult& association);

void WriteRirTrackAndId(float sim_time_sec, std::uint32_t cycle, const tracking::RirTrackState& track,
                        const session::RirRecognitionResult* result,
                        const session::RirFeatureMeasurementRecord* features,
                        bool has_truth, double category_accuracy);

void WriteRirSchedule(float sim_time_sec, std::uint32_t cycle, std::uint32_t planned,
                      std::uint32_t executed, float budget_sec, float consumed_sec,
                      std::uint32_t search_count, std::uint32_t track_count,
                      std::uint32_t ident_count);

void WriteRirOncePerSession(float sim_time_sec, std::uint32_t cycle);
void WriteRirCycleRunCount(float sim_time_sec, std::uint32_t cycle);
void WriteRirBeamScan(float sim_time_sec, std::uint32_t cycle, std::size_t wave_count,
                      float az_deg, float el_deg, bool designate);

bool TryExportRirAntennaPatternCsv(const config::hardware::RirAntennaConfig& antenna,
                                   const char* path);

}  // namespace runtime
}  // namespace remote_identification_radar

#endif
