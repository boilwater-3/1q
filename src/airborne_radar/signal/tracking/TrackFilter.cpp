// Copyright 2026. All Rights Reserved.
//
// Description: Minimal predictor/updater based track filter implementations.

#include "airborne_radar/signal/tracking/TrackFilter.h"

#include <algorithm>

namespace airborne_radar {
namespace signal {
namespace tracking {

PredictedTrackState IdentityTrackPredictor::Predict(
		const common::TargetFeature &input) const {
	return PredictedTrackState{input.current_track_speed, input.current_track_rcs,
														 input.current_track_acceleration};
}

SimpleTrackUpdater::SimpleTrackUpdater(TrackFilterConfig config)
		: config_(config) {}

common::TargetFeature SimpleTrackUpdater::Update(
		const PredictedTrackState &predicted,
		const TrackFilterContext &context) const {
	common::TargetFeature output(predicted.speed, predicted.rcs,
															 context.jamming_detected,
															 predicted.acceleration);

	if (!context.detection_succeeded) {
		output.current_track_speed =
				std::max(0.0f, predicted.speed * config_.speed_decay_ratio_on_loss);
		output.current_track_rcs =
				std::max(0.05f, predicted.rcs * config_.rcs_decay_ratio_on_loss);
	}

	if (context.jamming_detected) {
		output.current_track_acceleration =
				predicted.acceleration - config_.jamming_acceleration_penalty;
	} else {
		output.current_track_acceleration =
				predicted.acceleration +
				config_.stable_acceleration_gain * context.detection_margin_db;
	}

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
