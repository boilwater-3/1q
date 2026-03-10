// Copyright 2026. All Rights Reserved.
//
// Description: Minimal predictor/updater based track filter abstractions.

#ifndef AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_FILTER_H_
#define AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_FILTER_H_

#include "1q/airborne_radar/common/TargetFeature.h"

namespace airborne_radar {
namespace signal {
namespace tracking {

struct TrackFilterConfig {
	float speed_decay_ratio_on_loss{0.90f};
	float rcs_decay_ratio_on_loss{0.85f};
	float jamming_acceleration_penalty{0.5f};
	float stable_acceleration_gain{0.05f};
};

struct TrackFilterContext {
	bool detection_succeeded{false};
	bool jamming_detected{false};
	float detection_margin_db{0.0f};
};

struct PredictedTrackState {
	PredictedTrackState() = default;
	PredictedTrackState(float s, float r, float a) : speed(s), rcs(r), acceleration(a) {}

	float speed{0.0f};
	float rcs{0.0f};
	float acceleration{0.0f};
};

class ITrackPredictor {
public:
	virtual ~ITrackPredictor() = default;

	virtual PredictedTrackState Predict(
			const common::TargetFeature &input) const = 0;
};

class IdentityTrackPredictor final : public ITrackPredictor {
public:
	PredictedTrackState Predict(const common::TargetFeature &input) const override;
};

class ITrackUpdater {
public:
	virtual ~ITrackUpdater() = default;

	virtual common::TargetFeature Update(
			const PredictedTrackState &predicted,
			const TrackFilterContext &context) const = 0;
};

class SimpleTrackUpdater final : public ITrackUpdater {
public:
	explicit SimpleTrackUpdater(TrackFilterConfig config = {});

	common::TargetFeature Update(const PredictedTrackState &predicted,
															 const TrackFilterContext &context) const override;

	void UpdateConfig(TrackFilterConfig config);

private:
	TrackFilterConfig config_{};
};

class TrackFilter final {
public:
	explicit TrackFilter(TrackFilterConfig config = {});

	common::TargetFeature Filter(const common::TargetFeature &input,
															 const TrackFilterContext &context) const;

	void UpdateConfig(TrackFilterConfig config);

private:
	IdentityTrackPredictor predictor_{};
	SimpleTrackUpdater updater_{};
};

} // namespace tracking
} // namespace signal
} // namespace airborne_radar

#endif // AIRBORNE_RADAR_SIGNAL_TRACKING_TRACK_FILTER_H_
