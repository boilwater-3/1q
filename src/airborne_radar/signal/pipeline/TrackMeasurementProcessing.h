/**
 * @file TrackMeasurementProcessing.h
 * @brief 定义 SignalPipeline 量测构建与滤波写回的内部辅助函数。
 */

#ifndef AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_TRACK_MEASUREMENT_PROCESSING_H_
#define AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_TRACK_MEASUREMENT_PROCESSING_H_

#include <cstdint>
#include <vector>

#include "1q/airborne_radar/common/TargetFeature.h"
#include "1q/airborne_radar/common/JammingSemantics.h"
#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/detection/TargetGeometryResolver.h"
#include "airborne_radar/signal/tracking/GaussianTrackState.h"
#include "airborne_radar/signal/tracking/TrackFilter.h"
#include "airborne_radar/signal/tracking/TrackLifecycleTypes.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

struct TrackMeasurementBuildContext {
  const common::TargetFeatureList* input{nullptr};
  const association::AssociationResult* association_result{nullptr};
  const std::vector<std::uint8_t>* detection_succeeded{nullptr};
  const std::vector<std::uint64_t>* association_keys{nullptr};
  const std::vector<float>* detection_margin_db{nullptr};
  const std::vector<detection::ResolvedTargetGeometry>* target_geometry{nullptr};
  const std::vector<tracking::MeasurementCovariance>* measurement_covariances{nullptr};
  bool jamming_detected{false};
  common::JammingSemantic dominant_jamming_semantic{common::JammingSemantic::kNone};
  float jamming_severity{0.0f};
  std::vector<int>* measurement_slots{nullptr};
  std::vector<tracking::TrackMeasurement>* track_measurements{nullptr};
};

void BuildTrackMeasurementsPass(const TrackMeasurementBuildContext& context);

struct TrackFilterApplyContext {
  const common::TargetFeatureList* input{nullptr};
  common::TargetFeatureList* output{nullptr};
  const std::vector<std::uint8_t>* detection_succeeded{nullptr};
  const std::vector<float>* detection_margin_db{nullptr};
  bool jamming_detected{false};
  common::JammingSemantic dominant_jamming_semantic{common::JammingSemantic::kNone};
  float jamming_severity{0.0f};
  tracking::TrackFilter* track_filter{nullptr};
  const std::vector<int>* measurement_slots{nullptr};
  std::vector<tracking::TrackMeasurement>* track_measurements{nullptr};
};

void ApplyTrackFilterPass(const TrackFilterApplyContext& context);

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar

#endif  // AIRBORNE_RADAR_SRC_SIGNAL_PIPELINE_TRACK_MEASUREMENT_PROCESSING_H_
