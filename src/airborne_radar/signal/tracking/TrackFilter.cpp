// Copyright 2026. All Rights Reserved.
//
// Description: Minimal predictor/updater based track filter implementations.

#include "airborne_radar/signal/tracking/TrackFilter.h"

#include <algorithm>
#include <cmath>

namespace airborne_radar {
namespace signal {
namespace tracking {

namespace {

float VectorNorm3(float x, float y, float z) {
  return std::sqrt(x * x + y * y + z * z);
}

bool HasNonZero3(float x, float y, float z) {
  return x != 0.0f || y != 0.0f || z != 0.0f;
}

void NormalizeOrFallback(float x,
                         float y,
                         float z,
                         float &nx,
                         float &ny,
                         float &nz,
                         float fallback_x,
                         float fallback_y,
                         float fallback_z) {
  const float norm = VectorNorm3(x, y, z);
  if (norm > 1e-6f) {
    nx = x / norm;
    ny = y / norm;
    nz = z / norm;
    return;
  }

  nx = fallback_x;
  ny = fallback_y;
  nz = fallback_z;
}

}  // namespace

PredictedTrackState IdentityTrackPredictor::Predict(
		const common::TargetFeature &input) const {
	const bool has_velocity_axis =
			HasNonZero3(input.current_track_velocity_x,
							input.current_track_velocity_y,
							input.current_track_velocity_z);
	const float speed = has_velocity_axis
			? VectorNorm3(input.current_track_velocity_x,
							 input.current_track_velocity_y,
							 input.current_track_velocity_z)
			: input.current_track_speed;

	const bool has_accel_axis =
			HasNonZero3(input.current_track_acceleration_x,
							input.current_track_acceleration_y,
							input.current_track_acceleration_z);
	const float acceleration = has_accel_axis
			? VectorNorm3(input.current_track_acceleration_x,
							 input.current_track_acceleration_y,
							 input.current_track_acceleration_z)
			: input.current_track_acceleration;

	const float vx = has_velocity_axis ? input.current_track_velocity_x : speed;
	const float vy = has_velocity_axis ? input.current_track_velocity_y : 0.0f;
	const float vz = has_velocity_axis ? input.current_track_velocity_z : 0.0f;
	const float ax = has_accel_axis ? input.current_track_acceleration_x : acceleration;
	const float ay = has_accel_axis ? input.current_track_acceleration_y : 0.0f;
	const float az = has_accel_axis ? input.current_track_acceleration_z : 0.0f;

	return PredictedTrackState{speed, input.current_track_rcs, acceleration,
									 vx, vy, vz, ax, ay, az};
}

SimpleTrackUpdater::SimpleTrackUpdater(TrackFilterConfig config)
		: config_(config) {}

common::TargetFeature SimpleTrackUpdater::Update(
		const PredictedTrackState &predicted,
		const TrackFilterContext &context) const {
		common::TargetFeature output(predicted.velocity_x,
										 predicted.velocity_y,
										 predicted.velocity_z,
										 predicted.rcs,
										 predicted.acceleration_x,
										 predicted.acceleration_y,
										 predicted.acceleration_z);
		output.current_track_velocity_x = predicted.velocity_x;
		output.current_track_velocity_y = predicted.velocity_y;
		output.current_track_velocity_z = predicted.velocity_z;
		output.current_track_acceleration_x = predicted.acceleration_x;
		output.current_track_acceleration_y = predicted.acceleration_y;
		output.current_track_acceleration_z = predicted.acceleration_z;

	if (!context.detection_succeeded) {
		output.current_track_speed =
				std::max(0.0f, predicted.speed * config_.speed_decay_ratio_on_loss);
		output.current_track_rcs =
				std::max(0.05f, predicted.rcs * config_.rcs_decay_ratio_on_loss);

			float dir_vx = 1.0f;
			float dir_vy = 0.0f;
			float dir_vz = 0.0f;
			NormalizeOrFallback(predicted.velocity_x, predicted.velocity_y,
									 predicted.velocity_z,
									 dir_vx, dir_vy, dir_vz,
									 1.0f, 0.0f, 0.0f);
			output.current_track_velocity_x = dir_vx * output.current_track_speed;
			output.current_track_velocity_y = dir_vy * output.current_track_speed;
			output.current_track_velocity_z = dir_vz * output.current_track_speed;
	}

	if (context.jamming_detected) {
		output.current_track_acceleration =
				predicted.acceleration - config_.jamming_acceleration_penalty;
	} else {
		output.current_track_acceleration =
				predicted.acceleration +
				config_.stable_acceleration_gain * context.detection_margin_db;
	}

	float dir_ax = 1.0f;
	float dir_ay = 0.0f;
	float dir_az = 0.0f;
	NormalizeOrFallback(predicted.acceleration_x, predicted.acceleration_y,
							 predicted.acceleration_z,
							 dir_ax, dir_ay, dir_az,
							 1.0f, 0.0f, 0.0f);
	output.current_track_acceleration_x = dir_ax * output.current_track_acceleration;
	output.current_track_acceleration_y = dir_ay * output.current_track_acceleration;
	output.current_track_acceleration_z = dir_az * output.current_track_acceleration;

	return output;
}

void SimpleTrackUpdater::UpdateConfig(TrackFilterConfig config) {
	config_ = config;
}

TrackFilter::TrackFilter(TrackFilterConfig config) : updater_(config) {}

common::TargetFeature TrackFilter::Filter(
		const common::TargetFeature &input,
		const TrackFilterContext &context) const {
	const PredictedTrackState predicted = predictor_.Predict(input);
	return updater_.Update(predicted, context);
}

void TrackFilter::UpdateConfig(TrackFilterConfig config) {
	updater_.UpdateConfig(config);
}

} // namespace tracking
} // namespace signal
} // namespace airborne_radar
