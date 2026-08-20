#include "airborne_radar/signal/pipeline/TrackMeasurementProcessing.h"

#include <Eigen/Core>

#include "airborne_radar/signal/association/DataAssociation.h"
#include "airborne_radar/signal/pipeline/PipelineTargetUtils.h"

namespace airborne_radar {
namespace signal {
namespace pipeline {

namespace {

const association::AssociationMatch* FindAssociationMatch(
    const association::AssociationResult& result, std::size_t target_index) {
  for (const association::AssociationMatch& match : result.matches) {
    if (match.target_index == target_index) {
      return &match;
    }
  }
  return nullptr;
}

Eigen::Vector3f ResolveVelocityVector(const session::ArSceneTarget& target) {
  const Eigen::Vector3f velocity(target.velocity_x, target.velocity_y, target.velocity_z);
  if (velocity.squaredNorm() > 0.0f) {
    return velocity;
  }
  return Eigen::Vector3f::Zero();
}

}  // namespace

void BuildTrackMeasurementsPass(const session::ArSceneTargetList& input,
                                CycleExecutionScratch& scratch) {
  const std::size_t count = input.size();
  scratch.track_measurements.clear();
  if (scratch.track_measurements.capacity() < count) {
    scratch.track_measurements.reserve(count);
  }

  for (std::size_t i = 0; i < count; ++i) {
    if (scratch.detection_succeeded[i] == 0U) {
      continue;
    }

    const association::AssociationMatch* match =
        FindAssociationMatch(scratch.association_result, i);
    tracking::TrackMeasurement measurement;
    measurement.raw_measurement.source_index = i;
    measurement.raw_measurement.external_target_id = input[i].external_target_id;
    measurement.raw_measurement.target_name = input[i].target_name;
    measurement.raw_measurement.association_key = scratch.association_keys[i];
    measurement.raw_measurement.matched_existing_track = match != nullptr;
    measurement.raw_measurement.association_cost = match != nullptr ? match->cost : 0.0f;
    measurement.raw_measurement.used_position_association =
        scratch.association_result.used_position_association;
    measurement.raw_measurement.used_external_association_seeds =
        scratch.association_result.used_external_association_seeds;
    measurement.raw_measurement.detection_margin_db = scratch.detection_margin_db[i];
    measurement.raw_measurement.position = scratch.target_geometry[i].position_m;
    measurement.raw_measurement.measurement_covariance = scratch.measurement_covariances[i];
    scratch.measurement_slots[i] = static_cast<int>(scratch.track_measurements.size());
    scratch.track_measurements.push_back(measurement);
  }
}

void ApplyTrackFilterPass(const session::ArSceneTargetList& input,
                          tracking::TrackFilter& track_filter, CycleExecutionScratch& scratch) {
  const std::size_t count = scratch.output_state.size();
  for (std::size_t i = 0; i < count; ++i) {
    tracking::TrackFilterContext filter_context;
    filter_context.detection_succeeded = scratch.detection_succeeded[i] != 0U;
    filter_context.detection_margin_db = scratch.detection_margin_db[i];
    scratch.output_state[i] = track_filter.Filter(input[i], filter_context);

    const int measurement_slot = scratch.measurement_slots[i];
    if (measurement_slot < 0) {
      continue;
    }

    tracking::TrackMeasurement& measurement =
        scratch.track_measurements[static_cast<std::size_t>(measurement_slot)];
    measurement.filtered_feature.observed_speed = ResolveSpeedMagnitude(scratch.output_state[i]);
    measurement.filtered_feature.velocity = ResolveVelocityVector(scratch.output_state[i]);
    measurement.filtered_feature.rcs = scratch.output_state[i].rcs;
  }
}

}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
