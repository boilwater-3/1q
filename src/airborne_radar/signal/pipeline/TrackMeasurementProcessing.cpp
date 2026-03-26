#include "airborne_radar/signal/pipeline/TrackMeasurementProcessing.h"

#include <Eigen/Core>

namespace airborne_radar {
namespace signal {
namespace pipeline {
namespace internal {

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

float ResolveSpeedMagnitude(const common::TargetFeature& target) {
  const Eigen::Vector3f velocity(target.current_track_velocity_x, target.current_track_velocity_y,
                                 target.current_track_velocity_z);
  if (velocity.squaredNorm() > 0.0f) {
    return velocity.norm();
  }
  return target.current_track_speed;
}

Eigen::Vector3f ResolveVelocityVector(const common::TargetFeature& target) {
  const Eigen::Vector3f velocity(target.current_track_velocity_x, target.current_track_velocity_y,
                                 target.current_track_velocity_z);
  if (velocity.squaredNorm() > 0.0f) {
    return velocity;
  }
  return Eigen::Vector3f(target.current_track_speed, 0.0f, 0.0f);
}

}  // namespace

void BuildTrackMeasurementsPass(const TrackMeasurementBuildContext& context) {
  if (context.input == nullptr || context.association_result == nullptr ||
      context.detection_succeeded == nullptr || context.association_keys == nullptr ||
      context.detection_margin_db == nullptr || context.target_geometry == nullptr ||
      context.measurement_covariances == nullptr || context.measurement_slots == nullptr ||
      context.track_measurements == nullptr) {
    return;
  }

  const std::size_t count = context.input->size();
  context.track_measurements->clear();

  for (std::size_t i = 0; i < count; ++i) {
    if ((*context.detection_succeeded)[i] == 0U) {
      continue;
    }

    const association::AssociationMatch* match = FindAssociationMatch(*context.association_result, i);
    tracking::TrackMeasurement measurement;
    measurement.raw_measurement.source_index = i;
    measurement.raw_measurement.external_target_id = (*context.input)[i].external_target_id;
    measurement.raw_measurement.association_key = (*context.association_keys)[i];
    measurement.raw_measurement.matched_existing_track = match != nullptr;
    measurement.raw_measurement.association_cost = match != nullptr ? match->cost : 0.0f;
    measurement.raw_measurement.used_position_association =
        context.association_result->used_position_association;
    measurement.raw_measurement.used_external_association_seeds =
        context.association_result->used_external_association_seeds;
    measurement.raw_measurement.detection_margin_db = (*context.detection_margin_db)[i];
    measurement.raw_measurement.has_cartesian_position = (*context.target_geometry)[i].has_cartesian_position;
    measurement.raw_measurement.position = measurement.raw_measurement.has_cartesian_position
                                               ? (*context.target_geometry)[i].position_m
                                               : Eigen::Vector3f::Zero();
    measurement.raw_measurement.measurement_covariance = (*context.measurement_covariances)[i];
    measurement.filtered_feature.jamming_detected = context.jamming_detected;
    measurement.filtered_feature.dominant_jamming_semantic = context.dominant_jamming_semantic;
    measurement.filtered_feature.jamming_severity = context.jamming_severity;

    (*context.measurement_slots)[i] = static_cast<int>(context.track_measurements->size());
    context.track_measurements->push_back(measurement);
  }
}

void ApplyTrackFilterPass(const TrackFilterApplyContext& context) {
  if (context.input == nullptr || context.output == nullptr || context.detection_succeeded == nullptr ||
      context.detection_margin_db == nullptr || context.track_filter == nullptr ||
      context.measurement_slots == nullptr || context.track_measurements == nullptr) {
    return;
  }

  const std::size_t count = context.output->size();
  for (std::size_t i = 0; i < count; ++i) {
    tracking::TrackFilterContext filter_context;
    filter_context.detection_succeeded = (*context.detection_succeeded)[i] != 0U;
    filter_context.jamming_detected = context.jamming_detected;
    filter_context.dominant_jamming_semantic = context.dominant_jamming_semantic;
    filter_context.jamming_severity = context.jamming_severity;
    filter_context.detection_margin_db = (*context.detection_margin_db)[i];
    (*context.output)[i] = context.track_filter->Filter((*context.input)[i], filter_context);

    const int measurement_slot = (*context.measurement_slots)[i];
    if (measurement_slot < 0) {
      continue;
    }

    tracking::TrackMeasurement& measurement =
        (*context.track_measurements)[static_cast<std::size_t>(measurement_slot)];
    measurement.filtered_feature.observed_speed = ResolveSpeedMagnitude((*context.output)[i]);
    measurement.filtered_feature.velocity = ResolveVelocityVector((*context.output)[i]);
    measurement.filtered_feature.rcs = (*context.output)[i].current_track_rcs;
    measurement.filtered_feature.jamming_detected = context.jamming_detected;
    measurement.filtered_feature.dominant_jamming_semantic = context.dominant_jamming_semantic;
    measurement.filtered_feature.jamming_severity = context.jamming_severity;
  }
}

}  // namespace internal
}  // namespace pipeline
}  // namespace signal
}  // namespace airborne_radar
